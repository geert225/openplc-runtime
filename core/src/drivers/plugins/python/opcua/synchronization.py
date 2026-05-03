"""
OPC-UA ↔ PLC synchronization loop.

Drives the bidirectional value sync between the OPC-UA server's
Variable Nodes and the PLC program's variables. Reads PLC values
through args.debug_read; soft-writes client values through
args.debug_write (NOT debug_set — OPC-UA Write is a regular write
that the next scan cycle can overwrite, distinct from debugger
forcing which pins a value indefinitely).

Variables are addressed by (arr, elem) — the same tuple the editor
resolved against debug-map.json and wrote into opcua_config.json.
The plugin is variable-position-agnostic: it iterates whatever
(arr, elem) pairs the editor handed it and never opens the debug
map itself.
"""

import asyncio
from datetime import datetime, timezone
from typing import Any, Awaitable, Callable, Dict, Optional, Tuple

from asyncua import Server, ua

# Import local modules (handle both package and direct loading)
try:
    from .opcua_logging import log_debug, log_error, log_info
    from .opcua_memory import (
        TIME_DATATYPES,
        debug_read_value,
        debug_write_value,
        initialize_variable_cache,
    )
    from .opcua_types import VariableMetadata, VariableNode
    from .opcua_utils import (
        convert_value_for_opcua,
        convert_value_for_plc,
        map_plc_to_opcua_type,
    )
except ImportError:
    from opcua_logging import log_debug, log_error, log_info
    from opcua_memory import (
        TIME_DATATYPES,
        debug_read_value,
        debug_write_value,
        initialize_variable_cache,
    )
    from opcua_types import VariableMetadata, VariableNode
    from opcua_utils import (
        convert_value_for_opcua,
        convert_value_for_plc,
        map_plc_to_opcua_type,
    )


# Address tuple type alias for clarity.
Addr = Tuple[int, int]


class SynchronizationManager:
    """Bidirectional OPC-UA ↔ PLC value sync.

    Each cycle:
      1. OPC-UA → PLC: read every readwrite Variable Node's current
         OPC-UA value and forward it to args.debug_write if it
         changed since the last cycle. Force-writes are silently
         no-op'd by the runtime when the variable is currently forced
         (matches editor debugger semantics).
      2. PLC → OPC-UA: read every Variable Node's underlying PLC
         value via args.debug_read, decode per the configured
         datatype, and update the OPC-UA node so subscribed clients
         get notifications.
    """

    def __init__(
        self,
        args: Any,
        variable_nodes: Dict[Addr, VariableNode],
        server: Optional[Server] = None,
    ):
        """
        Args:
            args: PluginRuntimeArgs ctypes struct — exposes debug_read,
                  debug_write, debug_set, debug_size, debug_array_count.
            variable_nodes: Map (arr, elem) → VariableNode produced by
                            AddressSpaceBuilder.
            server: Optional asyncua Server for write_attribute_value
                    fast path with subscription notification timestamps.
        """
        self.args = args
        self.variable_nodes = variable_nodes
        self.server = server

        # Metadata cache (size + datatype) keyed by (arr, elem). Empty
        # until the first PLC-loaded cycle reaches initialize.
        self.variable_metadata: Dict[Addr, VariableMetadata] = {}

        # Last value seen per (arr, elem) — used to skip unchanged
        # OPC-UA → PLC writes. Populated lazily.
        self.opcua_value_cache: Dict[Addr, Any] = {}

        # Pre-filtered subset of variable_nodes that are writable.
        self._readwrite_nodes: Dict[Addr, VariableNode] = {}

        # Cycle timestamp for OPC-UA subscription notifications.
        self._cycle_timestamp: Optional[datetime] = None

        # Track no-PLC log to avoid spam.
        self._logged_no_plc_warning: bool = False

    # -----------------------------------------------------------------
    # Setup
    # -----------------------------------------------------------------

    async def initialize(self) -> bool:
        """One-time partition: split variable_nodes into readwrite vs
        readonly. Metadata cache is built lazily on the first cycle
        the PLC reports a non-zero array count."""
        try:
            self._readwrite_nodes = {
                addr: node
                for addr, node in self.variable_nodes.items()
                if node.access_mode == "readwrite"
            }

            log_debug(
                f"Sync manager: {len(self._readwrite_nodes)} readwrite, "
                f"{len(self.variable_nodes) - len(self._readwrite_nodes)} readonly"
            )
            return True
        except Exception as e:
            log_error(f"Failed to initialize sync manager: {e}")
            return False

    def _all_addresses(self) -> list:
        """Expand variable_nodes into the full list of (arr, elem)
        addresses, including every element of every array."""
        addrs: list = []
        for (base_arr, base_elem), node in self.variable_nodes.items():
            length = node.array_length or 0
            if length > 0:
                addrs.extend((base_arr, base_elem + i) for i in range(length))
            else:
                addrs.append((base_arr, base_elem))
        return addrs

    def _populate_metadata(self) -> None:
        """Build the (arr, elem) → metadata cache. Called when the
        plugin transitions from no-PLC to PLC-loaded."""
        addrs = self._all_addresses()
        # Build the datatype map from VariableNode (same datatype for
        # an array across all its elements).
        datatypes: Dict[Addr, str] = {}
        for (base_arr, base_elem), node in self.variable_nodes.items():
            length = node.array_length or 0
            if length > 0:
                for i in range(length):
                    datatypes[(base_arr, base_elem + i)] = node.datatype
            else:
                datatypes[(base_arr, base_elem)] = node.datatype

        self.variable_metadata = initialize_variable_cache(self.args, addrs, datatypes)
        # Reset value cache so the next cycle pushes everything.
        self.opcua_value_cache.clear()

    # -----------------------------------------------------------------
    # Top-level loop
    # -----------------------------------------------------------------

    async def run(
        self,
        is_running: Callable[[], bool],
        cycle_time_seconds: float,
    ) -> None:
        """Run the unified sync loop until is_running returns False."""
        log_info(f"Starting sync loop (cycle: {cycle_time_seconds * 1000:.0f}ms)")

        while is_running():
            try:
                # No PLC loaded yet — args.debug_array_count returns 0.
                # Skip syncing; clients will see the type-defaulted seed
                # values until a program loads.
                array_count = self.args.debug_array_count()
                if array_count == 0:
                    if not self._logged_no_plc_warning:
                        log_info("No PLC program loaded, sync paused")
                        self._logged_no_plc_warning = True
                    await asyncio.sleep(cycle_time_seconds)
                    continue

                # Transition no-PLC → PLC-loaded: rebuild metadata
                # cache so subsequent cycles know the byte sizes.
                if self._logged_no_plc_warning:
                    log_info("PLC program detected, resuming sync")
                    self._logged_no_plc_warning = False
                if not self.variable_metadata:
                    self._populate_metadata()
                    if not self.variable_metadata:
                        # All addresses still out-of-bounds — try again
                        # next cycle.
                        await asyncio.sleep(cycle_time_seconds)
                        continue

                self._cycle_timestamp = datetime.now(timezone.utc)
                await self.sync_opcua_to_runtime()
                await self.sync_runtime_to_opcua()
                await asyncio.sleep(cycle_time_seconds)

            except asyncio.CancelledError:
                log_debug("Sync loop cancelled")
                break
            except Exception as e:
                log_error(f"Error in sync loop: {e}")
                await asyncio.sleep(0.1)

        log_info("Sync loop stopped")

    # -----------------------------------------------------------------
    # OPC-UA → PLC (writes from clients)
    # -----------------------------------------------------------------

    async def sync_opcua_to_runtime(self) -> None:
        """Read every readwrite Variable Node's OPC-UA value and
        forward changes to args.debug_write (soft write — respects
        existing forces). Skips unchanged values via opcua_value_cache."""
        if not self._readwrite_nodes:
            return

        for (base_arr, base_elem), node in self._readwrite_nodes.items():
            try:
                opcua_value = await node.node.read_value()
                actual = self._extract_opcua_value(opcua_value)
                if actual is None:
                    continue

                length = node.array_length or 0
                if length > 0:
                    if not isinstance(actual, (list, tuple)):
                        continue
                    for i, elem_value in enumerate(actual[:length]):
                        addr = (base_arr, base_elem + i)
                        plc_value = convert_value_for_plc(node.datatype, elem_value)
                        if not self._has_value_changed(addr, plc_value):
                            continue
                        self._write_one(addr, node.datatype, plc_value)
                        self.opcua_value_cache[addr] = plc_value
                    continue

                # Scalar.
                addr = (base_arr, base_elem)
                plc_value = convert_value_for_plc(node.datatype, actual)
                if not self._has_value_changed(addr, plc_value):
                    continue
                self._write_one(addr, node.datatype, plc_value)
                self.opcua_value_cache[addr] = plc_value

            except Exception as e:
                log_error(f"Error reading OPC-UA variable ({base_arr},{base_elem}): {e}")

    def _write_one(self, addr: Addr, datatype: str, plc_value: Any) -> None:
        """Soft-write one (arr, elem) leaf via args.debug_write.

        TIME-family values arrive as (tv_sec, tv_nsec) tuples from
        convert_value_for_plc; recombine into the int64 nanosecond
        representation strucpp uses on the wire.
        """
        if datatype.upper() in TIME_DATATYPES and isinstance(plc_value, tuple):
            tv_sec, tv_nsec = plc_value
            plc_value = int(tv_sec) * 1_000_000_000 + int(tv_nsec)
        ok = debug_write_value(self.args, addr[0], addr[1], datatype, plc_value)
        if not ok:
            log_error(f"debug_write({addr[0]}, {addr[1]}) failed")

    # -----------------------------------------------------------------
    # PLC → OPC-UA (push to clients)
    # -----------------------------------------------------------------

    async def sync_runtime_to_opcua(self) -> None:
        """Read every Variable Node's PLC value via args.debug_read
        and update the OPC-UA node so subscribed clients see it."""
        if not self.variable_nodes:
            return

        for (base_arr, base_elem), node in self.variable_nodes.items():
            try:
                if node.array_length and node.array_length > 0:
                    await self._update_array_node(node, base_arr, base_elem)
                else:
                    value = debug_read_value(
                        self.args, base_arr, base_elem, node.datatype
                    )
                    if value is None:
                        continue
                    await self._update_opcua_node(node, value)
            except Exception as e:
                log_error(f"Failed to update node ({base_arr},{base_elem}): {e}")

    async def _update_opcua_node(self, node: VariableNode, value: Any) -> None:
        """Push one decoded scalar to its OPC-UA Variable Node."""
        opcua_value = convert_value_for_opcua(node.datatype, value)
        expected_type = map_plc_to_opcua_type(node.datatype)
        variant = ua.Variant(opcua_value, expected_type)
        data_value = ua.DataValue(
            Value=variant,
            StatusCode_=ua.StatusCode(ua.StatusCodes.Good),
            SourceTimestamp=self._cycle_timestamp,
            ServerTimestamp=datetime.now(timezone.utc),
        )
        if self.server:
            await self.server.write_attribute_value(node.node.nodeid, data_value)
        else:
            await node.node.write_value(variant)

    async def _update_array_node(
        self, node: VariableNode, base_arr: int, base_elem: int
    ) -> None:
        """Read every element of the array and push as a single
        OPC-UA array variant. Missing elements (None from debug_read)
        get a type-default placeholder so the array shape is preserved."""
        length = node.array_length or 0
        values = []
        for i in range(length):
            v = debug_read_value(
                self.args, base_arr, base_elem + i, node.datatype
            )
            if v is None:
                v = self._get_default_value(node.datatype)
            values.append(convert_value_for_opcua(node.datatype, v))

        expected_type = map_plc_to_opcua_type(node.datatype)
        variant = ua.Variant(values, expected_type)
        data_value = ua.DataValue(
            Value=variant,
            StatusCode_=ua.StatusCode(ua.StatusCodes.Good),
            SourceTimestamp=self._cycle_timestamp,
            ServerTimestamp=datetime.now(timezone.utc),
        )
        if self.server:
            await self.server.write_attribute_value(node.node.nodeid, data_value)
        else:
            await node.node.write_value(variant)

    # -----------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------

    @staticmethod
    def _get_default_value(datatype: str) -> Any:
        dtype = (datatype or "").upper()
        if dtype == "BOOL":
            return False
        if dtype in ("REAL", "LREAL"):
            return 0.0
        if dtype == "STRING":
            return ""
        return 0

    def _has_value_changed(self, addr: Addr, new_value: Any) -> bool:
        cached = self.opcua_value_cache.get(addr)
        if cached is None and addr not in self.opcua_value_cache:
            return True
        if isinstance(new_value, float) and isinstance(cached, float):
            return abs(new_value - cached) > 1e-6
        return new_value != cached

    @staticmethod
    def _extract_opcua_value(opcua_value: Any) -> Any:
        try:
            if hasattr(opcua_value, "Value"):
                return opcua_value.Value
            return opcua_value
        except Exception:
            return None

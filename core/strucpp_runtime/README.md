# strucpp_runtime

Runtime-side asset:

- `runtime_v4_entry.cpp` — small static C-linkage shim (~50 lines)
  compiled into every user `.so` by `scripts/compile.sh`. Defines
  `g_config`, exports `strucpp_get_config()` / `strucpp_set_locks()` /
  `strucpp_get_located_vars()` / `strucpp_get_located_var_count()`,
  and activates the C-linkage debug PDU exports from
  `debug_dispatch.hpp` (`STRUCPP_V4_DEBUG_EXPORTS_DEFINE`).
  Identical for every project — no per-project codegen.

The strucpp runtime headers are NOT vendored here. They ship with
each user-program upload under `core/generated/strucpp_runtime/include/`,
and `compile.sh` references them from there when compiling the `.so`.
The runtime executable (`plc_main`) does not include strucpp's full
header set — it uses a small layout-mirror at
`core/include/strucpp_abi.hpp` to walk the configuration via virtual
dispatch.

ABI contract: `core/include/strucpp_abi.hpp` must match the strucpp
runtime ABI used by the `.so`. When strucpp's ABI version bumps in a
breaking way, that file is the only thing that needs to update.

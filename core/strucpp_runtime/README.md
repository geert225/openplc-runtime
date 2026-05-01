# strucpp_runtime

Runtime-side STruC++ assets:

- `include/` — vendored snapshot of strucpp's runtime headers (the
  `iec_*.hpp` set + `debug_dispatch.hpp`). The version they were taken
  from is in `include/VERSION`. The runtime executable AND every user
  `.so` built by `scripts/compile.sh` include from this single copy, so
  ABI is automatically consistent.

- `runtime_v4_entry.cpp` — tiny static shim compiled into every user
  `.so`. Defines `g_config`, exports `strucpp_get_config()` /
  `strucpp_set_locks()`, activates the C-linkage debug PDU exports from
  `debug_dispatch.hpp` (`STRUCPP_V4_DEBUG_EXPORTS_DEFINE`). Does NOT
  ship from the editor — it's purely a runtime-side build asset.

When updating the strucpp version, replace `include/` wholesale and
update `include/VERSION`. Editors must be on a STruC++ release with a
compatible runtime ABI (same `ConfigurationInstance` /
`LocatedVar` / `Entry` layouts).

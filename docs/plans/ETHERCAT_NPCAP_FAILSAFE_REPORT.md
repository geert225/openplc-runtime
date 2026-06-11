# EtherCAT Plugin Fail-Safe on Windows (Missing Npcap) — Investigation Report

## 1. Problem statement

On Windows (MSYS2 build), if **Npcap is not installed**, the EtherCAT plugin fails to
load and that failure cascades into the whole runtime going to `PLC State: ERROR`,
so the PLC never reaches `RUNNING`. The desired behavior is "fail-safe": the plugin
should load successfully even without Npcap, sit idle, and only report an error
(e.g. "bus not running, install Npcap") *when the editor actually requests an
EtherCAT operation such as a bus scan*.

Observed log:
```
[ERROR] The EtherCAT plugin requires Npcap ... Please install Npcap and restart the runtime.
[ERROR] [PLUGIN] enabled native plugin 'ethercat' failed to load symbols
[ERROR] [PLUGIN]: Failed to load plugin configuration
[INFO]  PLC State: ERROR
[ERROR] State transition to RUNNING failed
```

## 2. Root cause (verified)

The failure happens at **library load time** (`dlopen`), **before the plugin's
`init()` is ever called** — not at scan time. The chain:

`plc_state_manager.cpp: load_plc_program()`
→ `plugin_driver_update_config(driver, "./plugins.conf")`
→ `native_plugin_get_symbols(plugin)` — `core/src/drivers/plugin_driver.c:1221`
→ `dlopen("./build/plugins/libethercat_plugin.so", RTLD_NOW)` — **fails, returns NULL**

```c
// core/src/drivers/plugin_driver.c:1236
void *handle = dlopen(plugin->config.path, RTLD_LOCAL | RTLD_NOW);
if (!handle) {
    log_error("Failed to load native plugin '%s': %s", ...);
#if defined(__CYGWIN__) || defined(_WIN32)
    if (strstr(plugin->config.name, "ethercat") != NULL)
        log_error("The EtherCAT plugin requires Npcap ...");   // line 1245
#endif
    free(native_bundle);
    return -1;                                                  // <-- symbol load fails here
}
```

**Why `dlopen` fails:** the plugin shared library is *statically linked against the
Npcap/WinPcap import libraries* in the EtherCAT CMake file:

```cmake
# core/src/drivers/plugins/native/ethercat/CMakeLists.txt  (generated MSYS.cmake block)
# line 109
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(WPCAP_LIB_PATH ${SOEM_SOURCE_DIR}/oshw/win32/wpcap/Lib/x64)
  target_link_libraries(soem PUBLIC
    ${WPCAP_LIB_PATH}/wpcap.lib       # import lib for wpcap.dll
    ${WPCAP_LIB_PATH}/Packet.lib      # import lib for Packet.dll
  )
...
```

`wpcap.lib` / `Packet.lib` are *import libraries* — they make the resulting `.so`
declare a hard, load-time dependency on `wpcap.dll` and `Packet.dll`. Those DLLs are
installed on the system **by Npcap** (in WinPcap-compatible mode). When Npcap is
absent, the Windows loader cannot resolve those DLLs, so `LoadLibrary`/`dlopen` of the
plugin fails outright. The plugin code never runs.

**Cascade to ERROR** (`core/src/drivers/plugin_driver.c`):
- An *enabled* native plugin whose symbols fail to load increments `load_failures`
  and the function returns `-1`:
  ```c
  // plugin_driver_update_config, ~line 419
  if (native_plugin_get_symbols(plugin) != 0) {
      if (plugin->config.enabled) {
          log_error("[PLUGIN] enabled native plugin '%s' failed to load symbols", ...);
          ++load_failures;
      }
  }
  ...
  return (load_failures > 0) ? -1 : 0;
  ```
- `load_plc_program()` treats that `-1` as fatal and forces `PLC_STATE_ERROR`
  (`core/src/plc_app/plc_state_manager.cpp` ~line 591):
  ```c
  if (plugin_driver_update_config(plugin_driver, "./plugins.conf") != 0) {
      log_error("[PLUGIN]: Failed to load plugin configuration");
      plc_state = PLC_STATE_ERROR;
      log_info("PLC State: ERROR");
      return -1;
  }
  ```

## 3. Key insight: the plugin is *already* designed to be lazy

This is the crucial finding. The EtherCAT plugin **never touches Npcap/SOEM during
`init()`**. `init()` (`ethercat_plugin.c:1183`) only:
- initializes its logger,
- copies runtime args,
- parses the JSON config,
- initializes mutexes,
- sets state to `IDLE`.

All actual pcap usage (`ecx_init` → `ecx_setupnic` → `pcap_open`) happens **lazily,
on demand**:
- `start_loop()` → `start_single_master()` → `ecat_master_open_and_scan()` →
  `ecx_init()` (`ethercat_master.c:150`), only when the PLC runs with EtherCAT
  configured;
- `execute_command("scan")` → `handle_scan_command()` → `ecx_init()`
  (`ethercat_plugin.c:1567`), only when the editor requests a scan.

Both of those paths **already return clean error strings on failure**:

```c
// ethercat_plugin.c:1567  (scan path — exactly the editor's bus-scan request)
if (!ecx_init(&scan_ctx, iface->valuestring)) {
    snprintf(response, response_size,
        "{\"error\":\"Failed to open interface '%s'\"}", iface->valuestring);
    return -1;
}
```

```c
// ethercat_master.c:152  (runtime path — already has Npcap-specific text)
if (!ecx_init(&inst->ecx_context, config->master.interface)) {
#if defined(__CYGWIN__) || defined(_WIN32)
    plugin_logger_error(logger,
        "Failed to initialize EtherCAT interface '%s'. "
        "Verify that Npcap (https://npcap.com) is installed ...");
#endif
    return -1;
}
```

And the command-dispatch layer already returns graceful JSON for a plugin that has no
`execute_command` symbol:

```c
// plugin_driver.c:1402  plugin_driver_execute_command()
if (plugin->config.type == PLUGIN_TYPE_NATIVE && plugin->native_plugin &&
    plugin->native_plugin->execute_command)
    return plugin->native_plugin->execute_command(command_json, response, response_size);

snprintf(response, response_size,
    "{\"error\":\"plugin '%s' does not support execute_command\"}", plugin_name);
return -1;
```

**Conclusion:** the desired "fail-safe" behavior is almost entirely implemented
already. The *only* thing breaking it is the **load-time DLL import dependency** on
`wpcap.dll`/`Packet.dll`, which kills `dlopen` before any of this lazy logic can run.
Fix that one thing and the rest of the desired UX falls out naturally.

## 4. Build / packaging context (for the fix)

- SOEM is a **git submodule**:
  `core/src/drivers/plugins/native/ethercat/libs/soem` (`.gitmodules`), built via
  `add_subdirectory()`.
- The bundled wpcap SDK (headers + `.lib`/`.a`) lives **inside the submodule** at
  `libs/soem/oshw/win32/wpcap/`. No external SDK is needed at build time — only the
  **runtime DLLs** (from Npcap) are needed at load time.
- Windows is built in **MSYS2 / MinGW GCC** (`.github/workflows/windows-installer.yml`,
  `install.sh` pacman block), packaged into an **Inno Setup** installer
  (`windows/setup.iss`) that bundles a full `msys64` tree + the runtime, launched by
  `windows/StartOpenPLC.bat`.
- The Windows NIC driver that calls pcap is SOEM's
  `libs/soem/oshw/win32/nicdrv.c` (`ecx_setupnic` → `pcap_open`, line 83+).
- `ec_find_adapters()` (used by the `list-interfaces` command, `ethercat_plugin.c:1627`)
  also lives in that win32 OSHW layer and also depends on wpcap.

> Note: the build is **MinGW GCC, not MSVC**. This rules out the easy MSVC
> `/DELAYLOAD` approach and shapes the recommended fix below.

## 5. Options to fix

### Option A — Runtime-load wpcap inside SOEM's win32 NIC driver  *(recommended)*

Stop linking `wpcap.lib`/`Packet.lib` as load-time imports. Instead resolve the small
set of pcap entry points SOEM actually uses at **first use**, via
`LoadLibraryA("wpcap.dll")` + `GetProcAddress`. If the DLL is absent, set a flag and
have `ecx_setupnic()` / `ec_find_adapters()` fail gracefully (return 0 / NULL with a
log line).

SOEM's win32 OSHW only touches a handful of symbols:
`pcap_open`, `pcap_close`, `pcap_sendpacket`, `pcap_next_ex`,
`pcap_findalldevs_ex` (+ `pcap_freealldevs`), and the related adapter-enumeration
calls. A thin function-pointer shim (one small `.c` file) covers all of them.

- **Result:** `libethercat_plugin.so` always loads → `init()` always succeeds →
  plugin sits `IDLE`. A later `scan`/`start` calls `ecx_init`, which fails cleanly
  with the existing "install Npcap" message. **Install Npcap later and it just works
  with no rebuild.** This is exactly the requested UX.
- **Cost:** patch a vendored submodule. Carry it as either:
  - a tracked patch applied during build (e.g. in `install.sh`/CMake before
    `add_subdirectory`), or
  - a fork of the submodule, or
  - an override: compile our **own** `nicdrv.c` (and the adapter-list TU) into the
    `soem` target instead of the stock ones, leaving the submodule pristine.
- **Effort:** moderate, localized, no behavior change on Linux (Linux OSHW uses raw
  AF_PACKET sockets, untouched).

### Option B — Make the loader tolerant of an enabled plugin that fails to load  *(recommended, complements A)*

Independent of A, harden the loader so a single native plugin failing to load does not
abort the entire runtime. In `plugin_driver_update_config` / `load_plc_program`, treat
a *symbol-load failure of an enabled native plugin* as **"degraded/unavailable"**
rather than fatal: keep the plugin instance in the array (with `native_plugin == NULL`),
log a warning, and let the PLC reach `RUNNING`. The existing
`plugin_driver_execute_command` path already returns a JSON error for a plugin with no
`execute_command`; we'd refine that message for the EtherCAT/degraded case
(e.g. `"EtherCAT unavailable: Npcap not installed"`).

- **Result:** boot never dies because of one plugin. Cheap insurance for *any* plugin,
  not just EtherCAT.
- **Limitation if used alone (without A):** the `.so` still has zero loaded symbols, so
  the plugin can never actually run, even after the user installs Npcap, without a
  restart — and arguably a rebuild isn't needed but the symbols would only resolve on a
  fresh `dlopen`. So B alone gives "boots + clean error" but not "works once Npcap is
  installed." **A + B together** gives the full desired behavior.

### Option C — Bundle/auto-install Npcap with the Windows installer  *(UX add-on, not a substitute)*

Add an optional step in `windows/setup.iss` to run the official **Npcap installer**
(downloaded or bundled per Npcap's license) during OpenPLC installation.

- Caveats: Npcap's license **prohibits redistributing `wpcap.dll`/`Packet.dll`
  standalone** — you must ship/run the official installer, and the **NDIS kernel driver**
  (not just the DLLs) is what actually grants raw access. So copying DLLs next to the
  runtime is *not* a valid fix. Bundling the official installer is valid but heavier
  (driver install, possible admin prompt — note the current installer is
  `PrivilegesRequired=lowest`).
- This improves first-run UX but does **not** remove the need for A+B (users who skip
  the Npcap step, or uninstall it, must still get a fail-safe runtime).

### Option D — Delay-load the import (MSVC-style)  *(not recommended here)*

`/DELAYLOAD:wpcap.dll` would defer resolution until first call. But the toolchain is
**MinGW GCC**, where GNU `ld` has no first-class delay-load for import libs (it needs a
`dlltool`-generated delay-load helper stub). More fragile and less portable than
Option A, which achieves the same effect explicitly.

## 6. Recommendation

Implement **Option A + Option B**:

1. **A** removes the load-time `wpcap.dll`/`Packet.dll` dependency by runtime-loading
   pcap inside SOEM's win32 NIC driver. This is the actual fix — boot succeeds, the
   plugin idles, scans report a clean "install Npcap" error, and everything works the
   moment Npcap is present (no rebuild). Prefer the "compile our own nicdrv TU"
   approach to keep the submodule pristine.
2. **B** makes the plugin loader resilient so no single plugin can force the runtime to
   `ERROR`, with an EtherCAT-specific "Npcap not installed" message surfaced through the
   existing `execute_command` JSON-error path.

Optionally layer **C** (bundle the official Npcap installer as an opt-in step) purely as
a first-run convenience.

Everything downstream of `dlopen` — lazy `ecx_init`, the scan handler's error JSON, the
Npcap-aware error text, graceful `execute_command` dispatch — **already exists** and
needs no new logic.

## 7. File reference

| Concern | File:line |
|---|---|
| `dlopen` failure + Npcap message | `core/src/drivers/plugin_driver.c:1236`, `:1245` |
| Enabled-plugin load failure → `-1` | `core/src/drivers/plugin_driver.c:~419` |
| Load failure → `PLC_STATE_ERROR` | `core/src/plc_app/plc_state_manager.cpp:~591` |
| `execute_command` dispatch (graceful JSON) | `core/src/drivers/plugin_driver.c:1402` |
| Command socket → plugin command | `core/src/plc_app/unix_socket.c:292` |
| Plugin `init()` (no pcap usage) | `core/src/drivers/plugins/native/ethercat/ethercat_plugin.c:1183` |
| Scan handler (lazy `ecx_init`, error JSON) | `ethercat_plugin.c:1543`, `:1567` |
| `list-interfaces` (also needs wpcap) | `ethercat_plugin.c:1625`, `:1627` |
| Runtime `ecx_init` + Npcap error text | `ethercat_master.c:150`, `:152` |
| wpcap import-lib linkage (root cause) | `core/src/drivers/plugins/native/ethercat/CMakeLists.txt:109` |
| SOEM win32 NIC driver (`pcap_open`) | `libs/soem/oshw/win32/nicdrv.c:83` |
| Windows CI build | `.github/workflows/windows-installer.yml` |
| Windows installer (Inno Setup) | `windows/setup.iss` |

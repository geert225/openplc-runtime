# Online Programming Plan — OpenPLC Runtime v4 (Linux targets)

> **Status:** brainstorm / pre-design.  Not committed; see *Decisions needed* at the end.

## Background

One of the most-requested OpenPLC features is **online programming** — change a small piece of the running program without stopping the PLC.  In interpreted-bytecode runtimes this is trivial: swap the program text between scan cycles.  We compile, so the unit of change is a freshly-built `.so`, and the trick is preserving program state across the swap.

This document maps the current runtime-v4 / strucpp architecture to that requirement, proposes a staged delivery, and enumerates the limitations and risks.

---

## What's already in place that helps

Three pieces of the existing architecture are *already* shaped for hot-swap; we just don't use them that way yet:

### 1. `.so` is loaded via `dlopen(..., RTLD_NOW)` and the runtime executable holds zero user-program state

Everything user-defined — POU struct instances, `IECVar<T>` storage, RETAIN vars, FB internals (timer counters, edge-detect flip-flops) — lives in the `.so`'s `.bss` / `.data`.  The runtime keeps only:

- Function pointers (`ext_strucpp_*`) into the `.so`'s exports (resolved in `image_tables.cpp::symbols_init`).
- A cached `ConfigurationInstance*` (`g_config_ptr`) returned by the `.so`'s `strucpp_get_config()`.
- Runtime-owned mutexes (`image_tables_mutex`, `global_vars_mutex`) whose addresses are *handed to* the `.so` via `strucpp_set_locks`.

Mechanically, unload-and-reload is one `dlclose(handle)` + one `dlopen(new_path, RTLD_NOW)` away.  The state-transplant problem is the hard part, not the dynamic-loading machinery.

### 2. The debug-table gives every leaf variable a stable nominal identity

strucpp's `debug-table-gen.ts` emits two artifacts per compile:

- **In the `.so`:** a `debug_arrays[][]` of `Entry { void* ptr; uint8_t tag; }` (declared in `debug_table.hpp`, defined in `generated_debug.cpp`).  Flat addressable by `(arrayIdx, elemIdx)`.
- **Out-of-band JSON `debugMap`:** maps fully-qualified IEC paths like `"INSTANCE0.speeds[5]"` → the `(arr, elem)` index in this build's table, plus the `TypeTag`.

Today the editor uses the JSON to do force / read.  The same JSON is exactly what we need to **transplant state by path** across the swap.  Path = stable nominal identity; `(arr, elem)` = positional address in the *current* program.

### 3. The scan loop already has a natural quiescence point

`plc_state_manager.cpp::plc_task_thread` does:

```
mutex_lock(image_tables_mutex)
  tracker_start
  io_cycle_pre              # only for the fastest task
  for each program: run()
  io_cycle_post             # only for the fastest task
  tracker_end
mutex_unlock(image_tables_mutex)
clock_nanosleep(TIMER_ABSTIME, next_wakeup)
```

The image-tables mutex is a recursive PI mutex.  The swap window is "after the last task unlocks, before any task next wakes" — already a no-touch zone for the program data.

---

## The swap mechanism, end-to-end

```
┌────────────────────────────────────────────────────────────────────┐
│ 1. Compile new program out-of-band at SCHED_OTHER nice 19          │
│    (or SCHED_IDLE) so it can't preempt SCHED_FIFO task threads.    │
│    Produce: libplc_<md5_new>.so + debugMap_new.json.               │
├────────────────────────────────────────────────────────────────────┤
│ 2. DRIFT GATE: compare debugMap_old vs debugMap_new (see below).   │
│    If REJECT → bail. If WARN → expose preview to editor user.      │
├────────────────────────────────────────────────────────────────────┤
│ 3. ARM the swap. Set an atomic `pending_swap = new_so_path`.       │
│    Plugins stay running; their state is untouched.                 │
├────────────────────────────────────────────────────────────────────┤
│ 4. SNAPSHOT. The fastest task, AFTER its `io_cycle_post`, BEFORE   │
│    `clock_nanosleep`, sees pending_swap != NULL and:               │
│      a. Suspends every OTHER task thread (cooperative — they       │
│         park at the top of their loop on a per-task condvar).      │
│      b. Walks every leaf in debug_arrays[] in the OLD .so, reads   │
│         IECVar<T>::{value_, forced_, forced_value_} via the        │
│         existing `strucpp_debug_read` plus a new                   │
│         `strucpp_debug_read_force_state` accessor, stores them     │
│         keyed by the debugMap_old path → an in-runtime arena.      │
│      c. Captures the `RetainVarInfo` table from each               │
│         `ProgramBase::getRetainVars()` and copies the retain bytes │
│         by leaf path as well.                                      │
│      d. Captures `__CURRENT_TIME_NS` (the per-.so monotonic time   │
│         the runtime owns via `ext_strucpp_advance_time`) — the     │
│         new .so resumes from the same time so TON / TOF / CTU      │
│         internal deadlines stay continuous.                        │
├────────────────────────────────────────────────────────────────────┤
│ 5. SWAP. Under image_tables_mutex:                                 │
│      a. image_tables_clear_null_pointers()                         │
│      b. dlclose(old handle)                                        │
│      c. dlopen(new .so, RTLD_NOW)                                  │
│      d. symbols_init(new pm) — re-resolves all ext_strucpp_*       │
│      e. image_tables_bind_located_vars() against new .so           │
├────────────────────────────────────────────────────────────────────┤
│ 6. RESTORE. For each leaf path in debugMap_old:                    │
│      - If new program has the same path AND same TypeTag:          │
│           write value_ via strucpp_debug_write,                    │
│           re-apply force via strucpp_debug_set(forcing=true)       │
│      - If type promoted (INT → DINT) within widening rules:        │
│           reinterpret + assign (lossless), reapply force.          │
│      - If path missing in new program (variable was deleted):      │
│           journal the dropped value, continue.                     │
│      - New paths not in old: keep their compile-time default.      │
│    Re-seed `strucpp_advance_time` with the saved CURRENT_TIME_NS   │
│    so timer FBs continue counting from where they were.            │
├────────────────────────────────────────────────────────────────────┤
│ 7. RESUME. Release suspended tasks. The fastest task continues     │
│    into its already-scheduled `next_wakeup` — drift visible to     │
│    plugins is bounded by the snapshot+swap+restore cost.           │
└────────────────────────────────────────────────────────────────────┘
```

---

## Drift gate — what changes are safe

The two `debugMap` JSONs are compared server-side before arming.  Buckets:

| Change                                                            | Old leaf path     | New leaf path     | Verdict                                                                                                                  |
| ----------------------------------------------------------------- | ----------------- | ----------------- | ------------------------------------------------------------------------------------------------------------------------ |
| **Identity preserved**                                            | `A.x: INT`        | `A.x: INT`        | ✅ transplant value + force                                                                                              |
| **Pure addition**                                                 | (absent)          | `A.y: INT`        | ✅ new leaf gets compile-time default                                                                                    |
| **Pure deletion**                                                 | `A.x: INT`        | (absent)          | ✅ value journaled and dropped                                                                                           |
| **Type widening, lossless**                                       | `A.x: INT`        | `A.x: DINT`       | ✅ cast + transplant — whitelist: SINT→INT→DINT→LINT, REAL→LREAL, signed-to-wider-signed only                            |
| **Type narrowing**                                                | `A.x: DINT`       | `A.x: INT`        | ⚠️ if `|value_| ≤ INT_MAX` proceed with cast, else REJECT (lossy)                                                        |
| **Type change cross-family**                                      | `A.x: REAL`       | `A.x: STRING`     | ❌ REJECT — no defined transplant                                                                                        |
| **Located-variable address remap**                                | `A.x AT %IX0.0`   | `A.x AT %IX0.1`   | ⚠️ accepted but live wire signal "moves" at swap instant — flag as DISRUPTIVE                                            |
| **Task topology change** (interval, priority, add, remove)        |                   |                   | ❌ REJECT — would invalidate step 4a's suspend list (threads no longer exist)                                            |
| **Resource topology change** (resources added / removed)          |                   |                   | ❌ REJECT — multi-resource swap is a different beast                                                                     |
| **Struct layout change** within named UDT (add / remove / reorder)|                   |                   | ⚠️ if every retained leaf path still resolves with same type → ACCEPT; else inspect per-field                            |
| **FB internal state reset** (timer FB's PT preset changed)        |                   |                   | ⚠️ ET (elapsed) preserved if path matches; new PT takes over at next `.EN` edge — flag as SEMANTIC-CHANGE                |
| **Code change inside a POU body** (no leaf changes)               |                   |                   | ✅ trivial — just swap the `.so`                                                                                         |

The compile-time `debugMap` makes all of these mechanically checkable without parsing source.  The editor can show the diff preview ("these 3 leaves will be added, this 1 will be dropped; values for the other 247 will be preserved") before the user commits.

---

## Limitations and risks

### Hard limitations

1. **PI mutex held across `dlclose`.**  `image_tables_mutex` is created by the runtime, but its address is handed to the `.so` via `strucpp_set_locks`.  If a stray plugin thread is blocked on it when we unload, we get a use-after-free on wakeup.  Mitigation: drain plugins from that mutex first (every plugin already exits its cycle window before `io_cycle_post` returns).
2. **C/C++ POUs compile into the `.so`.**  The user's `c_blocks_code.cpp` static state isn't reachable via the debug table — `static int counter` inside a user POU is lost across swap.  Either document this (users shouldn't keep state outside IEC variables) or have strucpp generate a per-POU retain blob.
3. **Pointers stored in IEC variables (REF_TO) become invalid across swap** because target addresses move when the new `.so`'s `.bss` lands at a different vmaddr.  Likely needs path-relative pointer canonicalization at snapshot time and re-resolution at restore.
4. **Plugins that opened sockets bound to IEC variable addresses** — Modbus mappings via image-tables, OPC-UA nodes that cache `&IECVar<T>::value_` — need to re-bind after the swap.  Extend the `_clear_null_pointers` / `_bind_located_vars` pattern: every plugin gets a `plugin_on_program_swap()` notification.
5. **PREEMPT_RT cycle jitter budget.**  Snapshot + swap + restore in step 4–6 has to fit in less than the fastest task's interval, OR we accept a single missed cycle.  With ~2000 leaves at ~200 ns each for `debug_read` + ~100 ns mutex ops, snapshot is ~600 µs; **`dlopen` on a Pi 4 from page cache is 5–20 ms** — that's the long pole.  For a 10 ms task we cannot do this in one cycle window.  Two paths:
   - **(a)** Accept a documented "one missed cycle" during swap.
   - **(b)** Keep both `.so`s mapped simultaneously and only switch the function-pointer table at the end of `io_cycle_post`, deferring `dlclose` of the old one to a later quiet period.

### Risks worth spiking early

- **PIC `.bss` aliasing.**  Two `dlopen`'d `.so`s with the same symbol names — the second `dlopen` won't ABI-collide if symbols are bound `RTLD_LOCAL` / `RTLD_DEEPBIND`.  strucpp's symbol resolution path currently assumes `RTLD_NOW` from the global namespace.  Worth a test before committing to "both `.so`s mapped at once" (the long-pole mitigation above).
- **Python plugin loader** keeps Python interpreter state.  The GIL holders and the embedded interpreter are runtime-owned (not in the user `.so`), so this should survive.  User-defined Python scripts that bind to specific variable names need a graceful re-resolution.
- **Debug-client liveness.**  If the editor is actively in a debugger session when the swap fires, the address tables it cached are stale.  Need a "program-revision token" that increments on every swap; the editor protocol re-resolves on mismatch.
- **MD5-based program identity.**  `strucpp_program_md5` is already exported and tracked.  The drift gate should use `md5_old ≠ md5_new` as the trigger gate (no swap on identical builds), but the **actual** drift comparison must use the structural `debugMap` diff — MD5 is only "did anything change at all".

---

## Staged delivery

Ship this in three releases rather than a v5 monolith:

### Stage 1 — Cold reload primitive

Runtime-side `/api/reload-program` endpoint that does the full swap but **stops the PLC first** (`PLC_STATE_STOPPED → swap → PLC_STATE_RUNNING`).  No state preservation.  Validates the dlopen / dlclose plumbing without committing to the harder semantics.  Editor can use it as "soft restart" — no power cycle, no plugin re-init, but no continuity either.

### Stage 2 — State transplant, identity-only

Adds the debugMap-driven snapshot / restore for the IDENTITY-PRESERVED + PURE-ADDITION + PURE-DELETION categories.  All other categories REJECT at the gate.  This is the version ~80 % of users would call "online programming" — change a POU body, tweak constants, add a variable, all preserve state.

### Stage 3 — Type / structure drift, located remap, FB internal preservation

The category whitelist expands; the editor shows the diff preview; users opt in to risky swaps.  This is where most of the engineering will actually live.

---

## Decisions needed before coding

1. **PREEMPT_RT jitter budget — which side of the trade?**  Is "one missed scan cycle during swap" acceptable, or is the "both `.so`s mapped simultaneously" approach required from the start?  That decision shapes step 5–6 of the swap mechanism.
2. **C/C++ POU static state — owned by the user or by us?**  If the answer is "users shouldn't keep state outside IEC variables," document it.  If it's "we should preserve it," strucpp needs a retain blob per user POU.
3. **Drift policy default for the gate** — strict (REJECT unless explicitly opted in) or permissive (warn-and-proceed)?  Working assumption: strict-by-default + editor-side "Force online change anyway" toggle.

---

## References in the current codebase

- `core/src/lib/strucpp_abi.hpp` — runtime-side ABI mirror of `ProgramBase` / `TaskInstance` / `ConfigurationInstance`.
- `core/src/plc_app/image_tables.{h,cpp}` — symbol resolution, `image_tables_bind_located_vars`, `image_tables_clear_null_pointers`.
- `core/src/plc_app/plcapp_manager.{h,c}` — `dlopen` / `dlsym` / `dlclose` wrapper.
- `core/src/plc_app/plc_state_manager.cpp::plc_task_thread` — per-task scan loop with mutex / `clock_nanosleep` rhythm.
- `core/src/plc_app/debug_handler.c` — wire protocol on top of `strucpp_debug_{set,read,write}`.
- `strucpp/src/runtime/include/debug_table.hpp` — `Entry` shape, `TypeTag` enum.
- `strucpp/src/runtime/include/debug_dispatch.hpp` — per-type `force_impl` / `unforce_impl` / `read_impl` / `write_impl`.
- `strucpp/src/backend/debug-table-gen.ts` — emits `generated_debug.cpp` *and* the out-of-band `debugMap` JSON keyed by IEC path.

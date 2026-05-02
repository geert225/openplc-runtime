#!/bin/bash
#
# compile.sh — build the user PLC program into core/build/new_libplc.so
#
# This script is the entry point the runtime's webserver calls after
# extracting an upload into core/generated/. The actual build rules
# live in scripts/Makefile.strucpp — invoking `make` lets us:
#
#   - run per-file compilation in parallel via `-j$(nproc)` (Pi 4 = 4×,
#     workstations more);
#   - reuse cached .o files automatically when ccache is installed,
#     so incremental rebuilds (one POU edited) drop from minutes to
#     a few seconds;
#   - adapt to whatever .cpp set STruC++'s codegen split emitted —
#     the Makefile uses `wildcard $(GENERATED_DIR)/*.cpp` rather than
#     a hard-coded list.
#
# The MatIEC-era files (Config0.c, Res0.c, debug.c, glueVars.c) are
# rejected with a clear error so stale uploads fail loudly instead of
# silently building against the old pipeline.

set -euo pipefail

GENERATED_DIR="core/generated"
RUNTIME_SHIM="core/strucpp_runtime/runtime_v4_entry.cpp"
RUNTIME_INC="$GENERATED_DIR/strucpp_runtime/include"

check_required_files() {
    if [ -f "$GENERATED_DIR/Config0.c" ] || [ -f "$GENERATED_DIR/glueVars.c" ]; then
        echo "[ERROR] core/generated contains MatIEC files (Config0.c / glueVars.c)." >&2
        echo "        This runtime no longer supports MatIEC programs."              >&2
        echo "        Re-export the project from a STruC++-aware editor build."      >&2
        exit 2
    fi

    local missing=()
    [ -f "$GENERATED_DIR/generated.hpp" ] || missing+=("$GENERATED_DIR/generated.hpp")
    [ -n "$(ls "$GENERATED_DIR"/*.cpp 2>/dev/null)" ] || missing+=("$GENERATED_DIR/*.cpp (at least one)")
    [ -f "$RUNTIME_SHIM" ]                || missing+=("$RUNTIME_SHIM")
    [ -d "$RUNTIME_INC" ]                 || missing+=("$RUNTIME_INC (directory)")

    if [ ${#missing[@]} -ne 0 ]; then
        echo "[ERROR] Missing required source files:" >&2
        printf '  %s\n' "${missing[@]}"               >&2
        exit 1
    fi
}

check_required_files

# Pick parallel job count. macOS / Linux differ on nproc availability;
# fall back to sysctl on Darwin and 4 elsewhere.
if command -v nproc >/dev/null 2>&1; then
    JOBS=$(nproc)
elif command -v sysctl >/dev/null 2>&1 && sysctl -n hw.ncpu >/dev/null 2>&1; then
    JOBS=$(sysctl -n hw.ncpu)
else
    JOBS=4
fi

exec make -j"$JOBS" -f scripts/Makefile.strucpp

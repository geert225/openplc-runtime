#!/bin/bash
#
# compile.sh — build the user PLC program into core/build/new_libplc.so
#
# STruC++ migration (Phase 5+): the editor uploads:
#   - core/generated/generated.cpp        (STruC++-emitted IEC body)
#   - core/generated/generated.hpp
#   - core/generated/generated_debug.cpp  (per-project pointer tables)
#   - optionally: core/generated/c_blocks_code.cpp + c_blocks.h
#
# The runtime contributes:
#   - core/strucpp_runtime/runtime_v4_entry.cpp  (static C-linkage shim)
#   - core/strucpp_runtime/include/              (vendored strucpp headers)
#
# The MatIEC-era files (Config0.c, Res0.c, debug.c, glueVars.c) are
# rejected with a clear error so stale uploads fail loudly instead of
# silently building against the old pipeline.

set -euo pipefail

GENERATED_DIR="core/generated"
RUNTIME_SHIM="core/strucpp_runtime/runtime_v4_entry.cpp"
# strucpp runtime headers ship with the user-program upload — the
# runtime itself does NOT vendor a copy.
RUNTIME_INC="$GENERATED_DIR/strucpp_runtime/include"
PYTHON_INCLUDE_PATH="core/src/plc_app/include"
PYTHON_LOADER_SRC="core/src/plc_app/python_loader.c"
BUILD_DIR="build"

mkdir -p "$BUILD_DIR"

check_required_files() {
    local missing=()

    if [ -f "$GENERATED_DIR/Config0.c" ] || [ -f "$GENERATED_DIR/glueVars.c" ]; then
        echo "[ERROR] core/generated contains MatIEC files (Config0.c / glueVars.c)." >&2
        echo "        This runtime no longer supports MatIEC programs."              >&2
        echo "        Re-export the project from a STruC++-aware editor build."      >&2
        exit 2
    fi

    [ -f "$GENERATED_DIR/generated.cpp" ]       || missing+=("$GENERATED_DIR/generated.cpp")
    [ -f "$GENERATED_DIR/generated.hpp" ]       || missing+=("$GENERATED_DIR/generated.hpp")
    [ -f "$GENERATED_DIR/generated_debug.cpp" ] || missing+=("$GENERATED_DIR/generated_debug.cpp")
    [ -f "$RUNTIME_SHIM" ]                      || missing+=("$RUNTIME_SHIM")
    [ -d "$RUNTIME_INC" ]                       || missing+=("$RUNTIME_INC (directory)")

    if [ ${#missing[@]} -ne 0 ]; then
        echo "[ERROR] Missing required source files:" >&2
        printf '  %s\n' "${missing[@]}"               >&2
        exit 1
    fi
}

check_required_files

CXXFLAGS=(
    # -O1 (not -O2): generated.cpp is mostly per-element IECVar
    # assignments and generated_debug.cpp is mostly constexpr-evaluated
    # address tables. -O2's aggressive inlining/vectorization buys very
    # little here while doubling compile time on a Pi 4 — large projects
    # were taking 10–15 min and tripping the editor's status-polling
    # timeout. Hot paths that benefit from -O2 (FB locks, image-tables
    # binding, scan-cycle housekeeping) live in runtime_v4_entry.cpp /
    # core/src/plc_app/, which are built with the project CMakeLists
    # at higher optimization. Arduino-targeted builds keep -Os/-O2 from
    # the Arduino platform.txt — those are constrained-resource
    # devices where the runtime tradeoff is different.
    # -pipe avoids temp files between cc1plus and as, modest 5–10% win.
    -std=c++17 -O1 -pipe -fPIC -Wall -Wno-unknown-pragmas -Wno-deprecated-declarations
    -I "$GENERATED_DIR" -I "$RUNTIME_INC" -I "$PYTHON_INCLUDE_PATH"
)

# Python POU stubs (`{external …}` blocks emitted by the editor) call
# `getpid()`, `create_shm_name()`, `python_block_loader()`. Their
# declarations live in iec_python.h. STruC++ stays Python-unaware —
# same as iec2c did — so the runtime force-includes the header into
# generated.cpp (this is exactly the trick MatIEC's compile.sh used:
# `-include iec_python.h` on the Config0.c invocation). Stubs without
# any Python POU pay nothing; the header is just declarations.
GENERATED_CXXFLAGS=("${CXXFLAGS[@]}" -include iec_python.h)

# On Cygwin/MSYS2, TCP/UDP communication blocks aren't supported — fall back
# to no-op stubs so blocks-using programs still compile.
EXTRA_OBJS=""
case "$(uname -s)" in
    CYGWIN*|MSYS*|MINGW*)
        cat > "$BUILD_DIR/comm_stubs.c" << 'STUB'
#include <stdint.h>
#include <stddef.h>
int connect_to_tcp_server(uint8_t *a, uint16_t b, int c) { (void)a; (void)b; (void)c; return -1; }
int send_tcp_message(uint8_t *a, size_t b, int c) { (void)a; (void)b; (void)c; return -1; }
int receive_tcp_message(uint8_t *a, size_t b, int c) { (void)a; (void)b; (void)c; return -1; }
int close_tcp_connection(int a) { (void)a; return -1; }
STUB
        gcc -O2 -fPIC -c "$BUILD_DIR/comm_stubs.c" -o "$BUILD_DIR/comm_stubs.o"
        EXTRA_OBJS="$BUILD_DIR/comm_stubs.o"
        ;;
esac

echo "[INFO] Compiling generated.cpp..."
g++ "${GENERATED_CXXFLAGS[@]}" -c "$GENERATED_DIR/generated.cpp" -o "$BUILD_DIR/generated.o"

echo "[INFO] Compiling generated_debug.cpp..."
g++ "${CXXFLAGS[@]}" -c "$GENERATED_DIR/generated_debug.cpp"  -o "$BUILD_DIR/generated_debug.o"

echo "[INFO] Compiling runtime_v4_entry.cpp..."
g++ "${CXXFLAGS[@]}" -c "$RUNTIME_SHIM"                       -o "$BUILD_DIR/runtime_v4_entry.o"

if [ -f "$GENERATED_DIR/c_blocks_code.cpp" ]; then
    echo "[INFO] Compiling c_blocks_code.cpp..."
    g++ "${CXXFLAGS[@]}" -c "$GENERATED_DIR/c_blocks_code.cpp" -o "$BUILD_DIR/c_blocks_code.o"
    EXTRA_OBJS="$EXTRA_OBJS $BUILD_DIR/c_blocks_code.o"
fi

if [ -f "$PYTHON_LOADER_SRC" ]; then
    echo "[INFO] Compiling python_loader.c..."
    gcc -O2 -fPIC -I "core/src/plc_app" -c "$PYTHON_LOADER_SRC" -o "$BUILD_DIR/python_loader.o"
    EXTRA_OBJS="$EXTRA_OBJS $BUILD_DIR/python_loader.o"
fi

echo "[INFO] Linking new_libplc.so..."
g++ -shared -fPIC -o "$BUILD_DIR/new_libplc.so" \
    "$BUILD_DIR/generated.o" \
    "$BUILD_DIR/generated_debug.o" \
    "$BUILD_DIR/runtime_v4_entry.o" \
    $EXTRA_OBJS \
    -lpthread -lrt

echo "[INFO] Build complete: $BUILD_DIR/new_libplc.so"

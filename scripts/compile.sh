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
    -std=c++17 -O2 -fPIC -Wall -Wno-unknown-pragmas -Wno-deprecated-declarations
    -I "$GENERATED_DIR" -I "$RUNTIME_INC"
)

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
g++ "${CXXFLAGS[@]}" -c "$GENERATED_DIR/generated.cpp"        -o "$BUILD_DIR/generated.o"

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

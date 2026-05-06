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

# Output path the Makefile drops new_libplc.so into. Also where any
# user-supplied VPP plugin .so will land so the runtime's plugin
# loader can pick them up under the same lookup rules.
BUILD_PATH="build"

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

# Build the program — actual rules live in scripts/Makefile.strucpp.
make -j"$(nproc)" -f scripts/Makefile.strucpp

# -----------------------------------------------------------------------
# Compile VPP plugin if source is present in the uploaded project
#
# The editor ships an optional vpp_plugin/ subtree alongside the IEC
# program when the project includes a VPP package. The plugin builds
# into BUILD_PATH (next to new_libplc.so) so the runtime's plugin
# loader picks it up under the same lookup rules as built-ins.
#
# Checksum cache: skip recompilation when the source hasn't changed
# AND a previous build's .so is still present. Saves ~20-60 s per
# upload on the slowest targets.
# -----------------------------------------------------------------------
VPP_PLUGIN_DIR="$GENERATED_DIR/vpp_plugin"
VPP_CHECKSUM_FILE="$VPP_PLUGIN_DIR/checksum.sha256"
# VPP outputs land in a dedicated subdir of BUILD_PATH so the cleanup
# glob below can scope itself to VPP-only artefacts. If a future built-in
# plugin ships as a .so dropped into BUILD_PATH directly, the old
# "$BUILD_PATH/lib*_plugin.so" glob would have rm'd it on every upload
# without a vpp_plugin subtree present.
VPP_OUTPUT_DIR="$BUILD_PATH/vpp"
VPP_CACHED_CHECKSUM="$VPP_OUTPUT_DIR/checksum.sha256"

if [ -d "$VPP_PLUGIN_DIR" ] && [ -f "$VPP_PLUGIN_DIR/Makefile" ]; then
    NEEDS_COMPILE=1
    mkdir -p "$VPP_OUTPUT_DIR"

    if [ -f "$VPP_CHECKSUM_FILE" ] && [ -f "$VPP_CACHED_CHECKSUM" ]; then
        if diff -q "$VPP_CHECKSUM_FILE" "$VPP_CACHED_CHECKSUM" > /dev/null 2>&1; then
            if ls "$VPP_OUTPUT_DIR"/lib*_plugin.so 1>/dev/null 2>&1; then
                echo "[INFO] VPP plugin source unchanged (checksum match), skipping recompilation"
                NEEDS_COMPILE=0
            fi
        fi
    fi

    if [ "$NEEDS_COMPILE" -eq 1 ]; then
        echo "[INFO] Compiling VPP plugin from $VPP_PLUGIN_DIR..."
        PLUGIN_INCLUDE="-I $(pwd)/core/src/drivers -I $(pwd)/core/src/drivers/plugins/native -I $(pwd)/core/src/drivers/plugins/native/cjson -I $(pwd)/core/src/plc_app -I $(pwd)/core/lib"
        make -C "$VPP_PLUGIN_DIR" \
            INCLUDE_DIRS="$PLUGIN_INCLUDE" \
            OUTPUT_DIR="$(pwd)/$VPP_OUTPUT_DIR" \
            RUNTIME_ROOT="$(pwd)"

        if [ -f "$VPP_CHECKSUM_FILE" ]; then
            cp "$VPP_CHECKSUM_FILE" "$VPP_CACHED_CHECKSUM"
        fi
        echo "[INFO] VPP plugin compiled successfully"
    fi
else
    # No VPP plugin in this upload — clean up the entire VPP output dir
    # so a stale .so doesn't get picked up by the loader. Scoping the rm
    # to VPP_OUTPUT_DIR (instead of a glob in BUILD_PATH) keeps cleanup
    # isolated from anything else that ends up in BUILD_PATH.
    if [ -d "$VPP_OUTPUT_DIR" ]; then
        echo "[INFO] No VPP plugin in upload, removing $VPP_OUTPUT_DIR"
        rm -rf "$VPP_OUTPUT_DIR"
    fi
fi

#!/bin/bash
# Creates a pair of linked pseudo-terminals for testing serial-based plugins
# (e.g. Profibus DP) without physical RS-485 hardware. One end is used as the
# plugin's configured serial device; the other end can be driven by a slave
# simulator (e.g. via `docker exec`).
set -euo pipefail

PORT_A="${1:-/var/run/runtime/ttyPB0}"
PORT_B="${2:-/var/run/runtime/ttyPB1}"

if ! command -v socat >/dev/null 2>&1; then
    echo "ERROR: socat is not installed" >&2
    exit 1
fi

rm -f "$PORT_A" "$PORT_B"

socat -d -d \
    pty,raw,echo=0,link="$PORT_A" \
    pty,raw,echo=0,link="$PORT_B" &

# Wait for both links to appear
for _ in $(seq 1 50); do
    [ -L "$PORT_A" ] && [ -L "$PORT_B" ] && break
    sleep 0.1
done

if [ ! -L "$PORT_A" ] || [ ! -L "$PORT_B" ]; then
    echo "ERROR: failed to create virtual serial port pair" >&2
    exit 1
fi

echo "Virtual serial port pair ready: $PORT_A <-> $PORT_B"

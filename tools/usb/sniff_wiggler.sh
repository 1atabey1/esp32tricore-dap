#!/bin/bash
#
# Capture the USB traffic of a working miniWiggler attach, so the DAP framing
# our own probe emits can be compared against a reference that demonstrably
# works.
#
# The miniWiggler V3.1 is an FT2232 behind an Infineon VID, so what crosses the
# wire is not DAP commands but MPSSE: FTDI opcodes carrying clock counts, bit
# payloads and GPIO writes.  That is *lower* level than DAP and therefore more
# useful - it shows the exact bits and every direction change.
#
# Run as root, because usbmon needs it:
#
#     sudo bash tools/usb/sniff_wiggler.sh [tas-debug-cli args...]
#
# Default action is `status`, which is a plain hot attach: the smallest amount
# of traffic that still proves the link.  Pass e.g. `read mSafetyApp foo` for a
# capture containing a memory read.
#
set -u

VID_PID="058b:0043"
OUT_DIR="${SUDO_USER:+/home/$SUDO_USER}/dap_captures"
OUT_DIR="${OUT_DIR:-/tmp}"
mkdir -p "$OUT_DIR"
STAMP=$(date +%Y%m%d_%H%M%S)
CAP="$OUT_DIR/wiggler_$STAMP.usbmon"

if [ "$(id -u)" -ne 0 ]; then
    echo "must run as root: usbmon is root-only" >&2
    exit 1
fi

# usbmon is a module in the WSL kernel but not loaded by default.
modprobe usbmon 2>/dev/null || true
if [ ! -d /sys/kernel/debug/usb/usbmon ]; then
    mount -t debugfs none /sys/kernel/debug 2>/dev/null || true
fi
if [ ! -d /sys/kernel/debug/usb/usbmon ]; then
    echo "usbmon is not available at /sys/kernel/debug/usb/usbmon" >&2
    exit 1
fi

# Which bus is the probe on?  usbmon has one file per bus.
BUS=$(lsusb -d "$VID_PID" | sed -E 's/^Bus ([0-9]+).*/\1/' | head -1)
if [ -z "$BUS" ]; then
    echo "no device $VID_PID on the USB bus" >&2
    exit 1
fi
BUSN=$((10#$BUS))
MON="/sys/kernel/debug/usb/usbmon/${BUSN}u"
# usbmon's *text* interface prints at most 32 data bytes per event, and this
# probe sends URBs up to ~100 bytes, so a text capture silently loses payload
# and every MPSSE command after the first cut is mis-framed.  tshark reads the
# binary interface and keeps everything, so prefer it when it is installed.
if command -v tshark >/dev/null 2>&1; then
    CAP="${CAP%.usbmon}.fields"
    echo "capturing usbmon$BUSN via tshark  ->  $CAP"
    tshark -i "usbmon$BUSN" -l -T fields \
        -e frame.time_relative -e usb.endpoint_address.direction \
        -e usb.endpoint_address.number -e usb.capdata \
        > "$CAP" 2>/dev/null &
    CAT_PID=$!
else
    echo "WARNING: tshark not installed - falling back to the usbmon text API," >&2
    echo "         which truncates payloads at 32 bytes. Install it with:" >&2
    echo "           sudo apt install -y tshark" >&2
    echo "capturing $MON  ->  $CAP"
    cat "$MON" > "$CAP" &
    CAT_PID=$!
fi
sleep 2

# Drive the reference probe as the invoking user, so uv and the project
# environment resolve the way they normally do.
ARGS=("$@")
if [ ${#ARGS[@]} -eq 0 ]; then
    ARGS=(status)
fi
RUN_AS="${SUDO_USER:-root}"
echo "running: tas-debug-cli ${ARGS[*]}  (as $RUN_AS)"
runuser -u "$RUN_AS" -- bash -lc "cd ~/repos/barcelonamain && \
    PATH=\$HOME/.local/bin:\$PATH timeout 120 uv run tas-debug-cli ${ARGS[*]} 2>&1 | tail -20"

sleep 1
kill "$CAT_PID" 2>/dev/null
wait "$CAT_PID" 2>/dev/null

chown "${SUDO_USER:-root}" "$CAP" 2>/dev/null || true
echo "captured $(wc -l < "$CAP") usbmon events into $CAP"

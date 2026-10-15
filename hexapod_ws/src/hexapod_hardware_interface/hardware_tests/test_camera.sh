#!/bin/bash
# Test the Raspberry Pi Camera Module 3: find it, take an autofocused photo and time a short video.
# Run on the Pi itself, not in the container: the Pi 5's cameras need Raspberry Pi OS's libcamera.
#
#   test_camera.sh [output.jpg]        # default ~/camera_test.jpg
#
# Then copy the photo to your PC to look at it:  scp raven@hexapod:camera_test.jpg .
set -uo pipefail
OUTPUT=${1:-$HOME/camera_test.jpg}
SENSOR=imx708   # Camera Module 3 (all variants)

fail() { echo "  FAIL  $1"; echo; echo "Result: FAIL"; exit 1; }

if ! command -v rpicam-hello >/dev/null; then
    fail "rpicam-apps aren't installed: sudo apt install rpicam-apps-lite"
fi

echo "Cameras libcamera can see:"
CAMERAS=$(rpicam-hello --list-cameras 2>&1)
echo "$CAMERAS" | sed 's/^/    /'
echo

# Lines look like "0 : imx708 [4608x2592 10-bit RGGB] (/base/axi/...)"
INDEX=$(echo "$CAMERAS" | sed -n "s/^ *\([0-9]\+\) : $SENSOR.*/\1/p" | head -n 1)
if [ -z "$INDEX" ]; then
    echo "  FAIL  No $SENSOR (Camera Module 3) found"
    echo "        Check the ribbon cable (contacts face the right way at both ends, latch closed),"
    echo "        and that /boot/firmware/config.txt has camera_auto_detect=1. Reboot after changes."
    echo; echo "Result: FAIL"; exit 1
fi
echo "  PASS  Camera Module 3 is camera $INDEX"
echo "$CAMERAS" | grep -q "^ *$INDEX : $SENSOR.*wide" && echo "        This is the wide-angle variant (120°), the URDF models the standard 66° lens"

echo
echo "Taking a photo with autofocus..."
if ! rpicam-still --camera "$INDEX" --nopreview --timeout 2000 --autofocus-on-capture -o "$OUTPUT" 2>/dev/null; then
    fail "rpicam-still failed, run it by hand to see the error"
fi
echo "  PASS  Saved $OUTPUT ($(du -h "$OUTPUT" | cut -f1)), check it's in focus and the right way up"

echo
echo "Recording 5 s of 1280x720 video to time the frame rate..."
PTS=$(mktemp)
ERR=$(mktemp)
trap 'rm -f "$PTS" "$ERR"' EXIT
# Raw YUV, not the default H.264: the Pi 5 has no H.264 hardware encoder and rpicam-apps-lite has no software one
rpicam-vid --camera "$INDEX" --nopreview --timeout 5000 --width 1280 --height 720 --framerate 30 \
    --codec yuv420 --save-pts "$PTS" -o /dev/null 2>"$ERR"
# --save-pts writes a header line, then one timestamp (ms) per frame
LOG=$(cat "$PTS")
FRAMES=$(echo "$LOG" | grep -c '^[0-9]')
if [ "$FRAMES" -lt 2 ]; then
    echo "  rpicam-vid said:"
    tail -n 5 "$ERR" | sed 's/^/    /'
    fail "No video frames recorded"
fi
FPS=$(echo "$LOG" | grep '^[0-9]' | awk 'NR == 1 { first = $1 } { last = $1 } END { printf "%.1f", (NR - 1) * 1000 / (last - first) }')
if awk "BEGIN { exit !($FPS >= 25) }"; then
    echo "  PASS  $FRAMES frames at $FPS fps (asked for 30)"
else
    echo "  WARN  $FRAMES frames at $FPS fps, asked for 30: low light (long exposures) or a CPU bottleneck?"
fi
echo
echo "Result: PASS"

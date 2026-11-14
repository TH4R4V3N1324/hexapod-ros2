#!/bin/bash
# Pair a Bluetooth game controller with the robot's Pi, so the robot can be driven without the
# Steam Deck. Run it on the Pi (the host, not the container) as your normal user.
#
#   pi/pair_controller.sh                     # pair the first controller found in pairing mode
#   pi/pair_controller.sh AA:BB:CC:DD:EE:FF   # pair this one
#
# Put the controller in pairing mode first:
#   Xbox (Bluetooth models)  hold the pair button on top until the logo flashes quickly
#   DualShock 4              hold Share + PS until the light bar flashes
#   DualSense                hold Create + PS until the lights around the touchpad flash
#   Switch Pro               hold the sync button on top
#   8BitDo and others        see the controller's manual (pick its X-input or Switch mode)
#
# The Pi trusts the controller, so after this it reconnects whenever the controller is turned on.
# USB controllers need none of this: plug them in.
set -euo pipefail
SCAN_SECONDS=${SCAN_SECONDS:-30}
# Names the common gamepads advertise; pass the address instead for one that isn't matched
NAME_PATTERN='Xbox|Wireless Controller|DualSense|DualShock|Pro Controller|8BitDo|Gamepad|Joy-Con'

sudo rfkill unblock bluetooth || true
bluetoothctl power on >/dev/null

# Scan in the background for the whole time, so the controller is still known when pairing
bluetoothctl --timeout "$((SCAN_SECONDS + 30))" scan on >/dev/null 2>&1 &
scan_pid=$!
trap 'kill "$scan_pid" 2>/dev/null || true' EXIT

mac=${1:-}
if [ -z "$mac" ]; then
    echo "Looking for a controller in pairing mode for up to $SCAN_SECONDS s..."
    paired=$(bluetoothctl devices Paired | awk '{print $2}')
    for _ in $(seq "$SCAN_SECONDS"); do
        mac=$(bluetoothctl devices | grep -E "$NAME_PATTERN" | awk '{print $2}' |
              grep -vxF -f <(printf '%s\n' "$paired") | head -n 1 || true)
        [ -n "$mac" ] && break
        sleep 1
    done
    if [ -z "$mac" ]; then
        echo "No controller found. Devices seen:"
        bluetoothctl devices
        echo "Check it is in pairing mode, or pass its address: $0 AA:BB:CC:DD:EE:FF"
        exit 1
    fi
fi

name=$(bluetoothctl info "$mac" | sed -n 's/^\s*Name: //p')
echo "Pairing ${name:-$mac} ($mac)"
bluetoothctl pair "$mac"
bluetoothctl trust "$mac"
bluetoothctl connect "$mac"
echo "Paired. The robot reads it as soon as it is connected (ros2 topic echo /joy to check)."

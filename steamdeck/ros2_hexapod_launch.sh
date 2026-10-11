#!/bin/bash
PORT=8765
HOTSPOT_HOST=192.168.50.1   # the robot's own Wi-Fi hotspot (pi/setup_pi.sh)

# On the robot's hotspot it is always at the same address
HOST=""
if (timeout 1 bash -c "echo >/dev/tcp/$HOTSPOT_HOST/$PORT") 2>/dev/null; then
    HOST=$HOTSPOT_HOST
else
    # Otherwise scan the local network for the bridge
    SUBNET=$(ip route | awk '/proto kernel/ && /192\.168/ {print $1}' | head -1)
    BASE=$(echo $SUBNET | cut -d'.' -f1-3)

    echo "Scanning $BASE.0/24 for port $PORT..."

    for i in $(seq 1 254); do
        (
            if (timeout 1 bash -c "echo >/dev/tcp/$BASE.$i/$PORT") 2>/dev/null; then
                echo "$BASE.$i" > /tmp/foxglove_bridge_ip
            fi
        ) &
    done
    wait

    HOST=$(cat /tmp/foxglove_bridge_ip 2>/dev/null)
    rm -f /tmp/foxglove_bridge_ip
fi

if [ -z "$HOST" ]; then
    echo "No bridge found, exiting."
    exit 1
fi

echo "Found bridge at: $HOST"

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo "No ROS 2 Jazzy in this distrobox, so the controller can't reach the robot."
    echo "Create the box with steamdeck/setup_distrobox.sh."
    exit 1
fi
source /opt/ros/jazzy/setup.bash

if [ -z "$DBUS_SESSION_BUS_ADDRESS" ]; then
    eval $(dbus-launch --sh-syntax --exit-with-session)
fi

export DISPLAY=${DISPLAY:-:0}
export WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-wayland-0}
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run/user/$(id -u)}

# game_controller_node reports the standard layout controller_teleop expects; plain joy_node
# gives the Deck's controls in their raw order (triggers on axes 2 and 5).
# The Deck's sticks jitter just off centre, and by default every change is published (up to
# 1 kHz): ~400 messages a second queued up over Wi-Fi and made the robot lag by about a second.
# Coalescing to 20 ms caps /joy at 50 Hz.
# A joy node from an earlier launch can still be running: Steam stops this script but not what it
# started inside the distrobox. Several publishers on /joy fight each other and add lag.
pkill -f "lib/joy/game_controller_node" 2>/dev/null || true
ros2 run joy game_controller_node --ros-args -p coalesce_interval_ms:=20 &
JOY_PID=$!
echo "ros2 joy node PID: $JOY_PID"

sleep 1
foxglove-studio --kiosk --url "foxglove://open?ds=foxglove-websocket&ds.url=ws://$HOST:$PORT" &
FOXGLOVE_PID=$!
echo "Foxglove Studio PID: $FOXGLOVE_PID"

echo "Press Ctrl+C to stop both."
trap "kill $JOY_PID $FOXGLOVE_PID 2>/dev/null" EXIT INT TERM
wait

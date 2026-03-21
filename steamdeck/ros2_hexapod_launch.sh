#!/bin/bash
PORT=8765

# Scan for bridge
SUBNET=$(ip route | awk '/proto kernel/ {print $1}' | head -1)
BASE=$(echo $SUBNET | cut -d'.' -f1-3)

echo "Scanning $BASE.0/24 for port $PORT..."

HOST=""
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

if [ -z "$HOST" ]; then
    echo "No bridge found, exiting."
    exit 1
fi

echo "Found bridge at: $HOST"

source /opt/ros/jazzy/setup.bash

if [ -z "$DBUS_SESSION_BUS_ADDRESS" ]; then
    eval $(dbus-launch --sh-syntax --exit-with-session)
fi

export DISPLAY=${DISPLAY:-:0}
export WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-wayland-0}
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run/user/$(id -u)}

ros2 run joy joy_node &
JOY_PID=$!
echo "ros2 joy node PID: $JOY_PID"

sleep 1
foxglove-studio --kiosk --url "foxglove://open?ds=foxglove-websocket&ds.url=ws://$HOST:$PORT" &
FOXGLOVE_PID=$!
echo "Foxglove Studio PID: $FOXGLOVE_PID"

echo "Press Ctrl+C to stop both."
trap "kill $JOY_PID $FOXGLOVE_PID 2>/dev/null" EXIT INT TERM
wait

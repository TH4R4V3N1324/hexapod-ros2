#!/usr/bin/env bash

set -e

# -------- CONFIG --------

ROS_DISTRO="jazzy"               # change if using another distro
WORKSPACE="$HOME/Hexapod/ros2/operator_ws"

# -------- SOURCE ROS --------

source /opt/ros/$ROS_DISTRO/setup.bash

# Source workspace if it exists

if [ -f "$WORKSPACE/install/setup.bash" ]; then
source "$WORKSPACE/install/setup.bash"
fi

# -------- CHECK FOXGLOVE BRIDGE --------

if ! ros2 pkg list | grep -q foxglove_bridge; then
echo "foxglove_bridge not found. Installing..."

```
sudo apt update
sudo apt install -y ros-$ROS_DISTRO-foxglove-bridge
```

fi

# -------- START NODES --------

echo "Starting ROS2 joy node..."
ros2 run joy joy_node &
JOY_PID=$!

echo "Starting Foxglove Bridge..."
ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
BRIDGE_PID=$!

sleep 3

echo "Launching Foxglove Studio..."

# Wait for foxglove_bridge websocket
for i in {1..30}; do
    if nc -z 127.0.0.1 8765 >/dev/null 2>&1; then
        break
    fi
    sleep 1
done

foxglove-studio ws://localhost:8765 &
FOX_PID=$!

echo "System running. Press Ctrl+C to stop."

trap "echo 'Stopping...'; kill $JOY_PID $BRIDGE_PID $FOX_PID; exit" SIGINT

wait


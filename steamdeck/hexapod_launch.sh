#!/bin/bash

exec > >(tee /tmp/hexapod_launch.log) 2>&1
unset LD_PRELOAD
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0
export XDG_RUNTIME_DIR=/run/user/1000
export PATH=$PATH:/home/deck/.local/bin

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

/home/deck/.local/bin/distrobox enter ubuntu --no-tty -- bash -c "$SCRIPT_DIR/ros2_hexapod_launch.sh"

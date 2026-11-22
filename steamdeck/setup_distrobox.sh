#!/bin/bash
# Create the Steam Deck's "jazzy" distrobox: Ubuntu 24.04 with ROS 2 Jazzy, matching the robot,
# plus the joy package (game_controller_node), controller_imu (built from operator_ws), Foxglove
# Studio and the Hexapod panel. Run it on
# the Deck itself (SteamOS, Konsole), not inside a box. Safe to run again; it also updates what's
# installed, including the panel from foxglove_panel/.
#
#   steamdeck/setup_distrobox.sh            # box named "jazzy", as hexapod_launch.sh expects
set -euo pipefail
export PATH=$PATH:$HOME/.local/bin
BOX=${1:-jazzy}

if ! distrobox list | awk -F'|' 'NR > 1 {gsub(/ /, "", $2); print $2}' | grep -qx "$BOX"; then
    distrobox create --yes --name "$BOX" --image docker.io/library/ubuntu:24.04
fi

distrobox enter "$BOX" --no-tty -- bash -s <<'EOF'
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive
sudo -E apt-get update
sudo -E apt-get install -y curl ca-certificates software-properties-common
sudo -E add-apt-repository -y universe

# ROS 2 apt repository, as in the Jazzy install guide
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
    version=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
        grep -F '"tag_name"' | awk -F'"' '{print $4}')
    curl -fsSL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version}.noble_all.deb"
    sudo -E apt-get install -y /tmp/ros2-apt-source.deb
fi

# Foxglove Studio: its package adds Foxglove's apt repository, so later runs update it from there
if ! dpkg -s foxglove-studio >/dev/null 2>&1; then
    curl -fsSL -o /tmp/foxglove-studio.deb \
        https://get.foxglove.dev/desktop/latest/foxglove-studio-latest-linux-amd64.deb
    sudo -E apt-get install -y /tmp/foxglove-studio.deb
fi

sudo -E apt-get update
# Node.js builds the Hexapod Foxglove panel (install_panel.sh); colcon, a compiler and SDL2 build
# controller_imu in operator_ws (controller tilt)
sudo -E apt-get install -y ros-jazzy-ros-core ros-jazzy-joy libsdl2-dev foxglove-studio nodejs npm \
    ros-dev-tools build-essential cmake pkg-config
sudo -E apt-get upgrade -y
echo "ROS: $(ls /opt/ros), Foxglove Studio $(dpkg-query -W -f '${Version}' foxglove-studio)"
EOF

# controller_imu reads the Deck's gyro for controller tilt; ros2_hexapod_launch.sh runs it from
# operator_ws/install. Only it is built: operator_bringup is for the operator PC's container.
OPERATOR_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../operator_ws" && pwd)"
distrobox enter "$BOX" --no-tty -- bash -c "set -eo pipefail
    source /opt/ros/jazzy/setup.bash
    cd '$OPERATOR_WS'
    colcon build --packages-select controller_imu"

distrobox enter "$BOX" --no-tty -- "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/install_panel.sh"

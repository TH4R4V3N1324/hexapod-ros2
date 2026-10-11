#!/bin/bash
# Create the Steam Deck's "jazzy" distrobox: Ubuntu 24.04 with ROS 2 Jazzy, matching the robot,
# plus the joy package (game_controller_node) and Foxglove Studio. Run it on the Deck itself
# (SteamOS, Konsole), not inside a box. Safe to run again; it also updates what's installed.
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
sudo -E apt-get install -y ros-jazzy-ros-core ros-jazzy-joy foxglove-studio
sudo -E apt-get upgrade -y
echo "ROS: $(ls /opt/ros), Foxglove Studio $(dpkg-query -W -f '${Version}' foxglove-studio)"
EOF

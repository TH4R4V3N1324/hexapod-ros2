#!/bin/bash
# Pull the latest code, then build and start the robot container (hexapod_ws/docker-compose.yml).
# Run at boot by hexapod-start.service. Offline, or if the pull or build fails, it starts the
# image it already has.
set -u
REPO=${HEXAPOD_REPO:-$HOME/hexapod-ros2}
cd "$REPO" || { echo "No repo at $REPO"; exit 1; }

# The network can take a while to come up after boot
for _ in $(seq 30); do
    timeout 10 git ls-remote -q origin HEAD >/dev/null 2>&1 && break
    sleep 2
done

# Fast-forward only, so local changes on the Pi are never merged or lost
if timeout 120 git pull --ff-only; then
    echo "Code at $(git log -1 --format='%h %s')"
else
    echo "git pull failed, starting the code already here ($(git log -1 --format='%h'))"
fi

cd hexapod_ws
# --pull fetches a newer ROS base image when there is one, which rebuilds everything and so also
# picks up new foxglove_bridge and ROS package releases. The old container keeps running meanwhile.
if docker compose build --pull; then
    docker compose up -d
else
    echo "Build failed, starting the last image"
    docker compose up -d --no-build
fi

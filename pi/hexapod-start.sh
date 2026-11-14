#!/bin/bash
# Start the robot container (hexapod_ws/docker-compose.yml) with the code and image already on the
# Pi. Run at boot by hexapod-start.service. It never pulls code: run pi/update.sh for that.
# The image is only built here if there isn't one yet, e.g. on a freshly set up Pi.
set -u
REPO=${HEXAPOD_REPO:-$HOME/hexapod-ros2}
cd "$REPO/hexapod_ws" || { echo "No workspace at $REPO/hexapod_ws"; exit 1; }

echo "Starting code at $(git -C "$REPO" log -1 --format='%h %s')"
docker compose up -d

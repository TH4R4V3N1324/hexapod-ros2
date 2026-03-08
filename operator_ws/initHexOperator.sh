#!/bin/bash
set -euo pipefail

# Start ROS2 nodes
docker compose up --build -d

# Wait for foxglove_bridge websocket port to come up.
for i in {1..30}; do
	if nc -z 127.0.0.1 8765 >/dev/null 2>&1; then
		break
	fi
	sleep 1
done

# Launch Foxglove with a datasource deep link.
FOXGLOVE_LINK='foxglove://open?ds=foxglove-websocket&ds.url=ws%3A%2F%2Flocalhost%3A8765'
if command -v xdg-open >/dev/null 2>&1; then
	xdg-open "$FOXGLOVE_LINK" >/dev/null 2>&1 || true
else
	foxglove-studio "$FOXGLOVE_LINK"
fi
#!/bin/bash
# Build the Hexapod Foxglove panel (foxglove_panel/) and install it into this machine's Foxglove
# Studio, replacing any older version. On the Deck it runs inside the jazzy distrobox
# (setup_distrobox.sh calls it); the box shares the Deck's home, where Foxglove keeps extensions.
set -euo pipefail
PANEL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../foxglove_panel" && pwd)"
EXTENSIONS="$HOME/.foxglove-studio/extensions"

cd "$PANEL_DIR"
npm ci --no-audit --no-fund
npm run package

FOXE=$(ls -t ./*.foxe | head -1)
NAME=$(basename "$FOXE" .foxe)
mkdir -p "$EXTENSIONS"
rm -rf "$EXTENSIONS"/raven.hexapod-panel-*
python3 -c "import sys, zipfile; zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])" "$FOXE" "$EXTENSIONS/$NAME"
echo "Installed $NAME. Restart Foxglove Studio and add the Hexapod panel to a layout."

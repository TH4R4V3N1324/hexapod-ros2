#!/bin/bash
# Update the robot's code and restart it with the new code. Run it on the Pi as your normal user.
#
#   pi/update.sh              # pull, update submodules, rebuild the image, restart the robot
#   pi/update.sh --no-build   # pull and update submodules only; the robot keeps running the old image
#
# It pulls fast-forward only, so local changes on the Pi are never merged or lost: if the Pi has
# commits of its own, or uncommitted changes the pull would overwrite, it stops and says so.
# The boot service (hexapod-start.service) never pulls; it starts whatever this last built.
set -euo pipefail

# Everything is in main, which runs once bash has read the whole file: the pull can replace this
# script while it runs, and bash otherwise reads a script as it goes
main() {
    local build=1
    case "${1:-}" in
        --no-build) build=0 ;;
        "") ;;
        *) echo "Usage: $0 [--no-build]"; exit 2 ;;
    esac

    local pi_dir repo
    pi_dir=$(cd "$(dirname "$0")" && pwd)
    repo=${HEXAPOD_REPO:-$(cd "$pi_dir/.." && pwd)}
    cd "$repo"

    echo "== Code (was $(git log -1 --format='%h %s'))"
    git pull --ff-only
    # sync first, in case a submodule's URL changed in .gitmodules
    git submodule sync --recursive
    git submodule update --init --recursive
    echo "Now at $(git log -1 --format='%h %s')"
    git submodule status

    if [ "$build" = 0 ]; then
        echo "Skipped the rebuild: the robot is still running the old image (run without --no-build to restart it)"
        return
    fi

    echo "== Image"
    cd hexapod_ws
    # --pull fetches a newer ROS base image when there is one, which rebuilds everything and so also
    # picks up new foxglove_bridge and ROS package releases. The old container keeps running meanwhile.
    docker compose build --pull
    echo "== Restarting the robot"
    docker compose up -d
    echo "Done"
}

main "$@"
exit

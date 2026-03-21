#!/usr/bin/env bash
 
 set -euo pipefail
 
 # -------- CONFIG --------
 
 ROS_DISTRO="${ROS_DISTRO:-jazzy}" # change if using another distro
 ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
 OPERATOR_WS="$ROOT_DIR/operator_ws"
 HEXAPOD_WS="$ROOT_DIR/hexapod_ws"
 
 REQUIRED_PACKAGES=(
 foxglove_bridge
 joy
 hexapod_interfaces
 hexapod_description
 )
 
 # -------- SOURCE ROS --------
 
 source /opt/ros/$ROS_DISTRO/setup.bash
 
 # Source operator workspace if it exists
 
 if [ -f "$OPERATOR_WS/install/setup.bash" ]; then
 source "$OPERATOR_WS/install/setup.bash"
 fi
 
 # Source robot workspace for custom interfaces/description if it exists.
 if [ -f "$HEXAPOD_WS/install/setup.bash" ]; then
 source "$HEXAPOD_WS/install/setup.bash"
 fi
 
 # Foxglove desktop expects ROS_PACKAGE_PATH for package:// asset resolution.
 if [ -n "${AMENT_PREFIX_PATH:-}" ]; then
 ros_package_path_entries=()
 OLD_IFS="$IFS"
 IFS=':'
 for prefix in $AMENT_PREFIX_PATH; do
 if [ -d "$prefix/share" ]; then
 ros_package_path_entries+=("$prefix/share")
 fi
 done
 IFS="$OLD_IFS"
 
 if [ "${#ros_package_path_entries[@]}" -gt 0 ]; then
 ROS_PACKAGE_PATH_FROM_AMENT="$(IFS=:; echo "${ros_package_path_entries[*]}")"
 if [ -n "${ROS_PACKAGE_PATH:-}" ]; then
 export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH_FROM_AMENT:$ROS_PACKAGE_PATH"
 else
 export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH_FROM_AMENT"
 fi
 fi
 fi
 
 # -------- CHECK FOXGLOVE BRIDGE --------
 
 if ! ros2 pkg list | grep -q foxglove_bridge; then
 echo "foxglove_bridge not found. Installing..."
 
 sudo apt update
 sudo apt install -y ros-$ROS_DISTRO-foxglove-bridge
 
 fi
 
 # Fail fast if required packages are not in the active overlay.
 missing_packages=()
 for pkg in "${REQUIRED_PACKAGES[@]}"; do
 if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
 missing_packages+=("$pkg")
 fi
 done
 
 if [ "${#missing_packages[@]}" -gt 0 ]; then
 echo
 echo "Missing ROS packages in current environment: ${missing_packages[*]}"
 echo "Expected overlays checked:"
 echo " - /opt/ros/$ROS_DISTRO"
 echo " - $OPERATOR_WS/install/setup.bash"
 echo " - $HEXAPOD_WS/install/setup.bash"
 echo
 echo "Build/install the missing packages, then rerun this script."
 if [ -d "$HEXAPOD_WS/src/hexapod_interfaces" ]; then
 echo "Hint: source /opt/ros/$ROS_DISTRO/setup.bash && cd $HEXAPOD_WS && colcon build --symlink-install"
 fi
 exit 1
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
 
 trap "echo 'Stopping...'; kill $JOY_PID $BRIDGE_PID $FOX_PID 2>/dev/null || true; exit" SIGINT
 
 wait

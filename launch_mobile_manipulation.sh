#!/usr/bin/env bash
# ============================================================
#  launch_mobile_manipulation.sh — Mobile manipulation bringup
#  OmniBot: Mecanum base + SO-101 arm + cameras
# ============================================================
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${SCRIPT_DIR}/robot_ws"

# Align ROS Domain ID
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}

# ---- Source ROS 2 base ----
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "[ERROR] ROS 2 setup not found at ${ROS_SETUP}."
    echo "        Set ROS_DISTRO env var to your distro (e.g. ROS_DISTRO=iron)."
    exit 1
fi

# shellcheck source=/dev/null
source "${ROS_SETUP}"

# ---- Source workspace overlay ----
WORKSPACE_SETUP="${WORKSPACE}/install/setup.bash"
if [[ ! -f "${WORKSPACE_SETUP}" ]]; then
    echo "[INFO] Workspace not built. Running colcon build first..."
    cd "${WORKSPACE}"
    colcon build --symlink-install
    echo "[INFO] Build complete."
fi

# shellcheck source=/dev/null
source "${WORKSPACE}/install/setup.bash"

# ---- Check for required packages ----
MISSING_PKGS=()

if ! ros2 pkg list 2>/dev/null | grep -q "^usb_cam$"; then
    MISSING_PKGS+=("usb_cam")
fi

if ! ros2 pkg list 2>/dev/null | grep -q "^rosbridge_server$"; then
    MISSING_PKGS+=("rosbridge_server")
fi

if [[ ${#MISSING_PKGS[@]} -gt 0 ]]; then
    echo ""
    echo "[WARNING] The following ROS 2 packages are not installed:"
    for pkg in "${MISSING_PKGS[@]}"; do
        echo "  - ${pkg}"
    done
    echo ""
    echo "Install them with:"
    echo "  sudo apt install \\"
    for pkg in "${MISSING_PKGS[@]}"; do
        echo "    ros-${ROS_DISTRO}-$(echo "${pkg}" | tr '_' '-') \\"
    done
    echo ""
    echo "Cameras and rosbridge will be unavailable until installed."
    echo ""
fi

# ---- Get local IP ----
LOCAL_IP=$(hostname -I | awk '{print $1}')

echo ""
echo "┌──────────────────────────────────────────────────────────┐"
echo "│   🤖  OmniBot Mobile Manipulation Bringup               │"
echo "│   Base + SO-101 Arm + Cameras                           │"
echo "│                                                          │"
echo "│   ROS Domain ID: ${ROS_DOMAIN_ID}                                    │"
echo "│   WebSocket:     ws://${LOCAL_IP}:9090              │"
echo "│   (Android app connects to the above address)           │"
echo "│                                                          │"
echo "│   Press Ctrl+C to stop all nodes                        │"
echo "└──────────────────────────────────────────────────────────┘"
echo ""

# ---- Launch mobile manipulation stack in background ----
ros2 launch omnibot_bringup mobile_manipulation.launch.py &
ROBOT_PID=$!

echo "[INFO] Mobile manipulation stack launched (PID: ${ROBOT_PID})"
echo "[INFO] Starting rosbridge for Android app connectivity..."
echo ""

# ---- Launch rosbridge (foreground) ----
# This script is the foreground process; Ctrl+C kills both
trap "echo ''; echo '[INFO] Shutting down...'; kill ${ROBOT_PID} 2>/dev/null; exit 0" SIGINT SIGTERM

if [[ -f "${SCRIPT_DIR}/launch_rosbridge.sh" ]]; then
    bash "${SCRIPT_DIR}/launch_rosbridge.sh"
else
    echo "[INFO] launch_rosbridge.sh not found — starting rosbridge directly."
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
fi

# Wait for background job
wait "${ROBOT_PID}"

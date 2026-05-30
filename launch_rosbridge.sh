#!/usr/bin/env bash
# ============================================================
#  launch_rosbridge.sh — Start ROSBridge WebSocket server
#  Allows the Android app to connect via ws://ROBOT_IP:9090
# ============================================================
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${SCRIPT_DIR}/robot_ws"

# ---- Deployment mode + DDS peer discovery ----
DEPLOY_ENV="${SCRIPT_DIR}/deployment.env"
NETWORK_ENV="${SCRIPT_DIR}/network.env"

if [[ -f "${DEPLOY_ENV}" ]]; then
    source "${DEPLOY_ENV}"
elif [[ -f "${NETWORK_ENV}" ]]; then
    source "${NETWORK_ENV}"
    DEPLOY_MODE="multi"
else
    DEPLOY_MODE="multi"
fi

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}

if [[ "${DEPLOY_MODE}" == "single" ]]; then
    unset ROS_STATIC_PEERS
else
    export ROS_STATIC_PEERS="${VLA_PC_IP:-${WORKSTATION_IP}};${PI_IP};${SIM_PC_IP}"
fi

# ---- Source ROS 2 base ----
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "[ERROR] ROS 2 setup not found at ${ROS_SETUP}."
    echo "        Set ROS_DISTRO env var to your distro (e.g. ROS_DISTRO=iron)."
    exit 1
fi

source "${ROS_SETUP}"

# ---- Check rosbridge is installed ----
if ! ros2 pkg list 2>/dev/null | grep -q "rosbridge_server"; then
    echo "[ERROR] rosbridge_server not found. Install it with:"
    echo "        sudo apt install ros-${ROS_DISTRO}-rosbridge-suite"
    exit 1
fi

# ---- Source workspace overlay if built ----
WORKSPACE_SETUP="${WORKSPACE}/install/setup.bash"
if [[ -f "${WORKSPACE_SETUP}" ]]; then
    source "${WORKSPACE_SETUP}"
fi

# ---- Show local IP for Android app config ----
LOCAL_IP=$(hostname -I | awk '{print $1}')

echo ""
echo "┌──────────────────────────────────────────────┐"
echo "│   🤖  Mecanum Wheel Robot — ROSBridge        │"
echo "│   WebSocket: ws://${LOCAL_IP}:9090        │"
echo "│   Domain ID: ${ROS_DOMAIN_ID}                               │"
echo "│   Set this IP in the Android app Settings    │"
echo "│   Press Ctrl+C to stop                       │"
echo "└──────────────────────────────────────────────┘"
echo ""

ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090

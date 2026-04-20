#!/usr/bin/env bash
# ============================================================
#  launch_teleop.sh — Single-click teleoperation launcher
#  Mecanum Wheel Robot (Yahboom / ROS 2)
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
    echo "[WARN] No deployment.env found — run: python deploy.py"
    DEPLOY_MODE="multi"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"

if [[ "${DEPLOY_MODE}" == "single" ]]; then
    unset ROS_STATIC_PEERS
    echo "[INFO] Deployment: single workstation (no DDS peers)"
else
    export ROS_STATIC_PEERS="${VLA_PC_IP:-${WORKSTATION_IP}};${PI_IP};${SIM_PC_IP}"
    echo "[INFO] Deployment: multi  |  DDS peers: ${ROS_STATIC_PEERS}"
fi

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
    echo "[ERROR] Workspace not built. Running colcon build first..."
    cd "${WORKSPACE}"
    colcon build --symlink-install
    echo "[INFO]  Build complete. Sourcing install..."
fi

# shellcheck source=/dev/null
source "${WORKSPACE}/install/setup.bash"

# ---- Launch ----
echo ""
echo "┌──────────────────────────────────────────────┐"
echo "│   🤖  Mecanum Wheel Robot — Teleoperation    │"
echo "│   Press Ctrl+C to stop                       │"
echo "└──────────────────────────────────────────────┘"
echo ""

ros2 launch omnibot_bringup robot_with_joy.launch.py

#!/usr/bin/env bash
# ============================================================
#  launch_simulation.sh — Run Gazebo + RViz simulation
#  Mecanum Wheel Robot workstation launcher
# ============================================================
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${SCRIPT_DIR}/robot_ws"

# ---- Deployment mode + DDS peer discovery ----
# deployment.env is written by `python deploy.py`.
# single mode: all nodes on one PC — no static peers needed.
# multi mode:  Pi + VLA PC + Sim PC — unicast peers set below.
DEPLOY_ENV="${SCRIPT_DIR}/deployment.env"
NETWORK_ENV="${SCRIPT_DIR}/network.env"

if [[ -f "${DEPLOY_ENV}" ]]; then
    source "${DEPLOY_ENV}"
elif [[ -f "${NETWORK_ENV}" ]]; then
    source "${NETWORK_ENV}"          # legacy fallback
    DEPLOY_MODE="multi"
else
    echo "[WARN] No deployment.env found — run: python deploy.py"
    DEPLOY_MODE="multi"
fi

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}

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

source "${ROS_SETUP}"

# ---- Build if necessary ----
WORKSPACE_SETUP="${WORKSPACE}/install/setup.bash"
if [[ ! -f "${WORKSPACE_SETUP}" ]]; then
    echo "[INFO] Building workspace..."
    cd "${WORKSPACE}"
    colcon build --symlink-install
fi

source "${WORKSPACE}/install/setup.bash"

# ---- Launch Simulation ----
echo ""
echo "┌──────────────────────────────────────────────┐"
echo "│   🤖  Mecanum Wheel Robot — Simulation       │"
echo "│   Domain ID: ${ROS_DOMAIN_ID}                               │"
echo "│   Press Ctrl+C to stop                       │"
echo "└──────────────────────────────────────────────┘"
echo ""

ros2 launch omnibot_bringup simulation.launch.py

#!/usr/bin/env bash
# ============================================================
#  launch_sim_pc.sh — Simulation workstation (PC2) launcher
#  OmniBot — multi-workstation deployment
#
#  Run this on PC2 (Isaac Sim / Gazebo machine) when using
#  multi-workstation mode (python deploy.py --mode multi).
#
#  What this starts:
#    • bev_stitcher_node  — stitches 4 base cameras into
#                           /camera/base/bev/image_raw (30 Hz)
#                           required by smolvla_node on PC1
#    • RViz               — optional visualisation
#
#  What this does NOT start:
#    • Isaac Sim          — start manually or via collect_episodes.py
#    • Gazebo             — use launch_simulation.sh for Gazebo-only runs
#    • VLA nodes          — those run on PC1 (launch_vla.sh / vla_desktop.launch.py)
#
#  Prerequisites:
#    1. Run `python deploy.py --mode multi` on this machine first.
#    2. Isaac Sim must be running with omni.isaac.ros2_bridge enabled, OR
#       Gazebo must be running via `ros2 launch omnibot_bringup simulation.launch.py`.
#    3. Same ROS_DOMAIN_ID (default 30) on all three machines.
#
#  Usage:
#    ./launch_sim_pc.sh           # with RViz
#    ./launch_sim_pc.sh --no-rviz # headless (no RViz)
# ============================================================
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${SCRIPT_DIR}/robot_ws"

# ---- Parse flags ----
USE_RVIZ="true"
for arg in "$@"; do
    case "${arg}" in
        --no-rviz) USE_RVIZ="false" ;;
    esac
done

# ---- Deployment mode + DDS peer discovery ----
DEPLOY_ENV="${SCRIPT_DIR}/deployment.env"
NETWORK_ENV="${SCRIPT_DIR}/network.env"

if [[ -f "${DEPLOY_ENV}" ]]; then
    source "${DEPLOY_ENV}"
elif [[ -f "${NETWORK_ENV}" ]]; then
    source "${NETWORK_ENV}"
    DEPLOY_MODE="multi"
else
    echo "[WARN] No deployment.env found — run: python deploy.py --mode multi"
    DEPLOY_MODE="multi"
fi

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}

if [[ "${DEPLOY_MODE}" == "single" ]]; then
    echo "[WARN] deployment.env says DEPLOY_MODE=single."
    echo "       launch_sim_pc.sh is intended for multi-workstation deployments."
    echo "       In single mode, just run ./launch_simulation.sh instead."
    echo "       Continuing anyway with localhost peers..."
    unset ROS_STATIC_PEERS
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

# ---- Show startup banner ----
echo ""
echo "┌──────────────────────────────────────────────────────┐"
echo "│   OmniBot — Simulation Workstation (PC2)             │"
echo "│   Domain ID: ${ROS_DOMAIN_ID}                                    │"
echo "│   DDS peers: ${ROS_STATIC_PEERS}  │"
echo "│                                                      │"
echo "│   Start Isaac Sim (separately) with:                 │"
echo "│     <isaac_sim>/python.sh data_engine/isaac_sim/     │"
echo "│       collect_episodes.py --episodes 100             │"
echo "│   Or run Gazebo with:                                │"
echo "│     ros2 launch omnibot_bringup simulation.launch.py │"
echo "│                                                      │"
echo "│   Press Ctrl+C to stop                              │"
echo "└──────────────────────────────────────────────────────┘"
echo ""

ros2 launch omnibot_bringup sim_pc.launch.py "rviz:=${USE_RVIZ}"

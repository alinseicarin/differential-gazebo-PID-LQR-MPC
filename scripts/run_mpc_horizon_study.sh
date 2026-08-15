#!/usr/bin/env bash
# Commission the MPC prediction horizon while holding every other setting fixed.
#
# Usage inside the container:
#   bash scripts/run_mpc_horizon_study.sh [result_directory]
#
# Optional environment variables:
#   MPC_HORIZONS="15 30 45"
#   MPC_HORIZON_GAZEBO_SEED=44
#   MPC_HORIZON_TRACK_PATH=/path/to/track.csv

set -eo pipefail

cd /home/ws
source /opt/ros/humble/setup.bash
source /home/ws/install/setup.bash

set -u

HORIZONS="${MPC_HORIZONS:-15 30 45}"
GAZEBO_SEED="${MPC_HORIZON_GAZEBO_SEED:-44}"
TRACK_PATH="${MPC_HORIZON_TRACK_PATH:-/home/ws/install/my_robot_controller/share/my_robot_controller/tracks/track_5_figure_eight.csv}"
RESULT_DIR="${1:-/home/ws/results/mpc_horizon_study_$(date +%Y%m%d_%H%M%S)}"

mkdir -p "${RESULT_DIR}"

for horizon in ${HORIZONS}; do
  if ! [[ "${horizon}" =~ ^[1-9][0-9]*$ ]]; then
    echo "Invalid positive MPC horizon: ${horizon}"
    exit 2
  fi

  echo "Running nominal figure-eight with MPC horizon ${horizon}"
  MPC_PREDICTION_HORIZON_STEPS="${horizon}" \
  PERTURBATION_CONTROLLER_FAMILY=mpc \
  PERTURBATION_SCENARIOS=nominal \
  PERTURBATION_GUI=false \
  PERTURBATION_GAZEBO_SEED="${GAZEBO_SEED}" \
  PERTURBATION_TRACK_PATH="${TRACK_PATH}" \
  PERTURBATION_TIMEOUT_SECONDS=90 \
    bash scripts/run_pid_perturbation_suite.sh \
      "${RESULT_DIR}/horizon_${horizon}"
done

python3 scripts/analyze_mpc_horizon_study.py "${RESULT_DIR}"
echo "MPC horizon study complete: ${RESULT_DIR}/horizon_summary.csv"

#!/usr/bin/env bash
# Re-run selected severe-campaign anomalies without extending the inferential
# dataset. These repetitions diagnose reproducibility only and must never be
# pooled with the predeclared ten-seed campaign.

set -eo pipefail

cd /home/ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  my_robot_description my_robot_controller
source /home/ws/install/setup.bash
python3 scripts/audit_project_consistency.py

set -u

RESULT_ROOT="${1:-/home/ws/results/severe_anomaly_recheck_$(date +%Y%m%d_%H%M%S)}"
REPETITIONS="${ANOMALY_RECHECK_REPETITIONS:-3}"
REFERENCE_CONFIG="/home/ws/install/my_robot_controller/share/my_robot_controller/config/trajectory_reference_severe.yaml"
TRACK="/home/ws/install/my_robot_controller/share/my_robot_controller/tracks/track_5_figure_eight.csv"

if ! [[ "${REPETITIONS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "Invalid anomaly recheck repetition count: ${REPETITIONS}" >&2
  exit 2
fi
if pgrep -x gzserver >/dev/null || pgrep -x gazebo >/dev/null; then
  echo "A Gazebo process is already running; stop it before the recheck" >&2
  exit 2
fi

export ROS_DOMAIN_ID="${ANOMALY_RECHECK_ROS_DOMAIN_ID:-93}"
export ROS_LOCALHOST_ONLY=1
mkdir -p "${RESULT_ROOT}"

{
  echo "SEVERE ANOMALY REPRODUCIBILITY CHECK"
  echo "role=diagnostic_post_hoc_not_for_inferential_pooling"
  echo "repetitions=${REPETITIONS}"
  echo "reference_config=${REFERENCE_CONFIG}"
  echo "track=${TRACK}"
  echo "fixed_observation_duration_s=30.0"
  echo "angular_pulse_start_delays_s=5.0;11.0;17.0"
  echo "cases=pid:1504:nominal mpc:1504:nominal pid:1507:nominal+angular_pulse_train lqr:1507:nominal+angular_pulse_train mpc:1507:nominal+left_wheel_loss_persistent"
} > "${RESULT_ROOT}/diagnostic_protocol.txt"

run_group()
{
  local repetition="$1"
  local family="$2"
  local seed="$3"
  local noise_seed="$4"
  local scenarios="$5"
  local output="${RESULT_ROOT}/repeat_$(printf '%02d' "${repetition}")/${family}_seed_${seed}"

  echo "RECHECK repetition=${repetition} controller=${family} seed=${seed} scenarios=${scenarios}"
  PERTURBATION_CONTROLLER_FAMILY="${family}" \
  PERTURBATION_SCENARIOS="${scenarios}" \
  PERTURBATION_GUI=false \
  PERTURBATION_GAZEBO_SEED="${seed}" \
  PERTURBATION_NOISE_SEED="${noise_seed}" \
  PERTURBATION_TRACK_PATH="${TRACK}" \
  PERTURBATION_REFERENCE_CONFIG_PATH="${REFERENCE_CONFIG}" \
  PERTURBATION_ANGULAR_PULSE_START_DELAYS="5.0;11.0;17.0" \
  PERTURBATION_FIXED_OBSERVATION_DURATION="30.0" \
  PERTURBATION_TIMEOUT_SECONDS=90 \
    bash scripts/run_pid_perturbation_suite.sh "${output}"
}

for repetition in $(seq 1 "${REPETITIONS}"); do
  run_group "${repetition}" pid 1504 19004 "nominal"
  run_group "${repetition}" mpc 1504 19004 "nominal"
  run_group "${repetition}" pid 1507 19007 "nominal angular_pulse_train"
  run_group "${repetition}" lqr 1507 19007 "nominal angular_pulse_train"
  run_group "${repetition}" mpc 1507 19007 "nominal left_wheel_loss_persistent"
done

python3 scripts/analyze_severe_anomaly_recheck.py "${RESULT_ROOT}"
echo "Anomaly recheck complete: ${RESULT_ROOT}/recheck_summary.csv"

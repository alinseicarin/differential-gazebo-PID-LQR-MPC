#!/usr/bin/env bash
# Run the paired PID--TVLQR--MPC experiment matrix in isolated Gazebo sessions.
#
# The default matrix is intentionally the final, relatively expensive campaign:
# ten paired seeds, three smooth nominal tracks, and the full robustness suite
# on the figure-eight. Use the selectors below for a small development run.
#
# Example smoke test:
#   COMPARISON_REPETITIONS=1 \
#   COMPARISON_NOMINAL_TRACKS="straight" \
#   COMPARISON_ROBUSTNESS_SCENARIOS="" \
#     bash scripts/run_controller_comparison.sh /tmp/comparison_smoke

set -eo pipefail

cd /home/ws
source /opt/ros/humble/setup.bash
# A final campaign must never execute stale binaries from an older source tree.
colcon build --symlink-install --packages-select \
  my_robot_description my_robot_controller
source /home/ws/install/setup.bash

# Abort before the first trial if physical dimensions, frequencies, common
# parameters, generated tracks, or controller launch interfaces have drifted.
python3 scripts/audit_project_consistency.py

set -u

RESULT_DIR="${1:-/home/ws/results/controller_comparison_$(date +%Y%m%d_%H%M%S)}"
CONTROLLER_TEXT="${COMPARISON_CONTROLLERS-pid lqr mpc}"
NOMINAL_TRACK_TEXT="${COMPARISON_NOMINAL_TRACKS-straight curve circle}"
ROBUSTNESS_TRACK="${COMPARISON_ROBUSTNESS_TRACK-figure_eight}"
ROBUSTNESS_SCENARIOS="${COMPARISON_ROBUSTNESS_SCENARIOS-nominal angular_pulse angular_constant left_wheel_loss command_delay localization_noise}"
REPETITIONS="${COMPARISON_REPETITIONS:-10}"
BASE_GAZEBO_SEED="${COMPARISON_BASE_GAZEBO_SEED:-500}"
BASE_NOISE_SEED="${COMPARISON_BASE_NOISE_SEED:-9000}"
GUI="${COMPARISON_GUI:-false}"
RESUME="${COMPARISON_RESUME:-true}"
ROS_DOMAIN="${COMPARISON_ROS_DOMAIN_ID:-91}"

if ! [[ "${REPETITIONS}" =~ ^[1-9][0-9]*$ ]] ||
  ! [[ "${BASE_GAZEBO_SEED}" =~ ^[0-9]+$ ]] ||
  ! [[ "${BASE_NOISE_SEED}" =~ ^[0-9]+$ ]] ||
  ! [[ "${ROS_DOMAIN}" =~ ^[0-9]+$ ]] ||
  ((ROS_DOMAIN > 232)) ||
  [[ "${GUI}" != "true" && "${GUI}" != "false" ]] ||
  [[ "${RESUME}" != "true" && "${RESUME}" != "false" ]]
then
  echo "Invalid repetitions, seed, GUI, or resume selector" >&2
  exit 2
fi

# Keep the thesis graph off the network and away from ordinary ROS sessions.
export ROS_DOMAIN_ID="${ROS_DOMAIN}"
export ROS_LOCALHOST_ONLY=1

# Reusing an already-running Gazebo instance could silently preserve a robot,
# physics state, or ROS topic from outside the paired protocol.
if pgrep -x gzserver >/dev/null || pgrep -x gazebo >/dev/null; then
  echo "A Gazebo process is already running; stop it before the campaign" >&2
  exit 2
fi

read -r -a controllers <<< "${CONTROLLER_TEXT}"
read -r -a nominal_tracks <<< "${NOMINAL_TRACK_TEXT}"
if [[ "${#controllers[@]}" -eq 0 ]]; then
  echo "At least one controller must be selected" >&2
  exit 2
fi

for controller in "${controllers[@]}"; do
  case "${controller}" in
    pid|lqr|mpc) ;;
    *) echo "Unknown controller: ${controller}" >&2; exit 2 ;;
  esac
done

TRACK_ROOT="/home/ws/install/my_robot_controller/share/my_robot_controller/tracks"
CONFIG_ROOT="/home/ws/install/my_robot_controller/share/my_robot_controller/config"

# Hash every runtime-relevant source, launch, configuration, script, and track.
# Resume is allowed only when this fingerprint and the complete protocol agree,
# preventing old and new implementations from being mixed in one dataset.
SOURCE_FINGERPRINT="$({
  find src/my_robot_controller src/my_robot_description scripts \
    -type f -not -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum
  sha256sum .devcontainer/Dockerfile generate_tracks.py track_*.csv
} | sha256sum | awk '{print $1}')"
PROTOCOL_SIGNATURE="$(printf '%s\n' \
  "${CONTROLLER_TEXT}" "${NOMINAL_TRACK_TEXT}" "${ROBUSTNESS_TRACK}" \
  "${ROBUSTNESS_SCENARIOS}" "${REPETITIONS}" "${BASE_GAZEBO_SEED}" \
  "${BASE_NOISE_SEED}" "${ROS_DOMAIN}" "${SOURCE_FINGERPRINT}" | \
  sha256sum | awk '{print $1}')"
if [[ -f "${RESULT_DIR}/protocol.txt" ]]; then
  previous_signature="$(awk -F= '$1 == "protocol_signature" {print $2}' \
    "${RESULT_DIR}/protocol.txt")"
  if [[ "${previous_signature}" != "${PROTOCOL_SIGNATURE}" ]]; then
    echo "Refusing to mix a changed protocol or source tree into ${RESULT_DIR}" >&2
    echo "Use a new result directory, or restore the archived implementation" >&2
    exit 2
  fi
fi
mkdir -p "${RESULT_DIR}/protocol_tracks" "${RESULT_DIR}/protocol_configs"

# Keep the exact runtime source beside the data even if the workspace contained
# uncommitted changes when the campaign was launched.
tar --exclude='*/__pycache__/*' -czf "${RESULT_DIR}/protocol_source.tar.gz" \
  src/my_robot_controller src/my_robot_description scripts \
  .devcontainer/Dockerfile generate_tracks.py track_*.csv

# Archive the exact configuration files selected at campaign start. Runtime
# metadata records their hashes, and the analyzer also compares every common
# actuator, completion, timing, and safety parameter across controller families.
for controller in "${controllers[@]}"; do
  case "${controller}" in
    pid) source_config="${CONFIG_ROOT}/pid_cascade.yaml" ;;
    lqr) source_config="${CONFIG_ROOT}/lqr.yaml" ;;
    mpc) source_config="${CONFIG_ROOT}/mpc.yaml" ;;
  esac
  cp "${source_config}" "${RESULT_DIR}/protocol_configs/${controller}.yaml"
done
cp "${CONFIG_ROOT}/trajectory_reference.yaml" \
  "${RESULT_DIR}/protocol_configs/trajectory_reference.yaml"

# The source line is 20 m long. Freeze a 5 m copy once so every controller and
# repetition consumes byte-identical waypoints of a scale comparable to the
# other nominal tracks.
STRAIGHT_TRACK="${RESULT_DIR}/protocol_tracks/track_straight_5m.csv"
head -n 101 "${TRACK_ROOT}/track_1_straight.csv" > "${STRAIGHT_TRACK}"

track_path()
{
  case "$1" in
    straight) echo "${STRAIGHT_TRACK}" ;;
    curve) echo "${TRACK_ROOT}/track_2_curve.csv" ;;
    circle) echo "${TRACK_ROOT}/track_4_circle.csv" ;;
    figure_eight) echo "${TRACK_ROOT}/track_5_figure_eight.csv" ;;
    *) echo "Unknown smooth comparison track: $1" >&2; return 2 ;;
  esac
}

{
  echo "PID--TVLQR--MPC paired comparison protocol"
  echo "controllers=${CONTROLLER_TEXT}"
  echo "nominal_tracks=${NOMINAL_TRACK_TEXT}"
  echo "robustness_track=${ROBUSTNESS_TRACK}"
  echo "robustness_scenarios=${ROBUSTNESS_SCENARIOS}"
  echo "repetitions=${REPETITIONS}"
  echo "base_gazebo_seed=${BASE_GAZEBO_SEED}"
  echo "base_noise_seed=${BASE_NOISE_SEED}"
  echo "ros_domain_id=${ROS_DOMAIN}"
  echo "source_fingerprint=${SOURCE_FINGERPRINT}"
  echo "protocol_signature=${PROTOCOL_SIGNATURE}"
  echo "controller order rotates once per repetition"
  echo "negative paired difference means the first controller has less error"
} > "${RESULT_DIR}/protocol.txt"

run_suite()
{
  local controller="$1"
  local track="$2"
  local scenarios="$3"
  local repetition="$4"
  local gazebo_seed="$5"
  local noise_seed="$6"
  local output="${RESULT_DIR}/run_$(printf '%02d' "${repetition}")_seed_${gazebo_seed}/${controller}/track_${track}"

  if [[ "${RESUME}" == "true" && -s "${output}/summary.csv" ]]; then
    local summary_complete=1
    local expected_scenario
    for expected_scenario in ${scenarios}; do
      if ! awk -F, -v scenario="${expected_scenario}" \
        'NR > 1 && $3 == scenario {found = 1} END {exit !found}' \
        "${output}/summary.csv"
      then
        summary_complete=0
        break
      fi
    done
    if [[ "${summary_complete}" -eq 1 ]]; then
      echo "SKIP completed ${controller} ${track} repetition ${repetition}"
      return
    fi
    echo "RESUME incomplete ${controller} ${track} repetition ${repetition}"
  fi

  echo "START ${controller} ${track} repetition=${repetition} seed=${gazebo_seed}"
  PERTURBATION_CONTROLLER_FAMILY="${controller}" \
  PERTURBATION_SCENARIOS="${scenarios}" \
  PERTURBATION_GUI="${GUI}" \
  PERTURBATION_GAZEBO_SEED="${gazebo_seed}" \
  PERTURBATION_NOISE_SEED="${noise_seed}" \
  PERTURBATION_TRACK_PATH="$(track_path "${track}")" \
  PERTURBATION_TIMEOUT_SECONDS=90 \
    bash scripts/run_pid_perturbation_suite.sh "${output}"
}

for repetition in $(seq 1 "${REPETITIONS}"); do
  gazebo_seed=$((BASE_GAZEBO_SEED + repetition - 1))
  noise_seed=$((BASE_NOISE_SEED + repetition - 1))

  # Rotating the order prevents one family from always being evaluated first
  # while retaining a deterministic, auditable protocol.
  for offset in "${!controllers[@]}"; do
    index=$(((repetition - 1 + offset) % ${#controllers[@]}))
    controller="${controllers[${index}]}"

    for track in "${nominal_tracks[@]}"; do
      run_suite "${controller}" "${track}" nominal \
        "${repetition}" "${gazebo_seed}" "${noise_seed}"
    done
    if [[ -n "${ROBUSTNESS_SCENARIOS}" ]]; then
      run_suite "${controller}" "${ROBUSTNESS_TRACK}" \
        "${ROBUSTNESS_SCENARIOS}" "${repetition}" \
        "${gazebo_seed}" "${noise_seed}"
    fi
  done
done

python3 scripts/analyze_controller_comparison.py "${RESULT_DIR}"
echo "Comparison campaign complete: ${RESULT_DIR}/group_summary.csv"

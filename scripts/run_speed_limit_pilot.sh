#!/usr/bin/env bash
# Characterize the current virtual robot at increasing actual circle speeds.

set -eo pipefail

cd /home/ws

RESULT_ROOT="${1:-/tmp/speed_limit_pilot}"
SPEED_TEXT="${SPEED_LIMIT_PILOT_SPEEDS:-0.4 0.6 0.8 1.0}"
GUI="${SPEED_LIMIT_PILOT_GUI:-false}"
CURVATURE_GAIN="${SPEED_LIMIT_PILOT_CURVATURE_GAIN:-0.0}"
REPETITIONS="${SPEED_LIMIT_PILOT_REPETITIONS:-3}"

mkdir -p "${RESULT_ROOT}"
read -r -a SPEEDS <<< "${SPEED_TEXT}"
if [[ "${#SPEEDS[@]}" -eq 0 ]]; then
  echo 'At least one pilot speed is required' >&2
  exit 2
fi

for speed in "${SPEEDS[@]}"; do
  if ! [[ "${speed}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "Invalid pilot speed: ${speed}" >&2
    exit 2
  fi
  speed_tag="${speed//./p}"
  speed_directory="${RESULT_ROOT}/speed_${speed_tag}"
  reference_config="${speed_directory}/trajectory_reference.yaml"
  mkdir -p "${speed_directory}"

  printf '%s\n' \
    '/**:' \
    '  ros__parameters:' \
    '    search_window: 20' \
    "    reference_linear_velocity: ${speed}" \
    "    curvature_speed_gain: ${CURVATURE_GAIN}" \
    '    endpoint_slowdown_distance: 0.0' \
    '    maximum_reference_curvature: 5.0' \
    '    trajectory_spatial_step: 0.01' \
    '    maximum_reference_linear_acceleration: 0.5' \
    '    maximum_reference_linear_deceleration: 0.1' \
    '    maximum_reference_angular_velocity: 1.5' \
    > "${reference_config}"

  echo "SPEED_LIMIT_PILOT speed=${speed} m/s"
  PID_BENCHMARK_GUI="${GUI}" \
  PID_BENCHMARK_TRACKS='circle' \
  PID_BENCHMARK_PROFILES='cascade' \
  PID_BENCHMARK_REPETITIONS="${REPETITIONS}" \
  PID_BENCHMARK_BASE_SEED=800 \
  PID_BENCHMARK_REFERENCE_CONFIG_PATH="${reference_config}" \
    bash scripts/run_pid_benchmarks.sh "${speed_directory}"
done

python3 scripts/analyze_speed_limit_pilot.py "${RESULT_ROOT}"

#!/usr/bin/env bash
# Run the controller-independent robustness suite with PID, LQR, or MPC.
#
# Usage inside the container:
#   bash scripts/run_pid_perturbation_suite.sh [result_directory]
#
# Optional:
#   PERTURBATION_GUI=true PERTURBATION_SCENARIOS="nominal angular_pulse_train" ...
#   PERTURBATION_CONTROLLER_FAMILY=mpc selects mpc.yaml and the LTV-MPC launch.

set -eo pipefail

cd /home/ws
source /opt/ros/humble/setup.bash
source /home/ws/install/setup.bash

set -u

CONTROLLER_FAMILY="${PERTURBATION_CONTROLLER_FAMILY:-pid}"
RESULT_DIR="${1:-/home/ws/results/${CONTROLLER_FAMILY}_perturbation_suite}"
SCENARIOS="${PERTURBATION_SCENARIOS:-nominal angular_pulse_train angular_constant left_wheel_loss left_wheel_loss_persistent command_delay localization_noise localization_yaw_bias}"
COMMON_GUI="${PERTURBATION_GUI:-false}"
COMMON_TRACK="${PERTURBATION_TRACK_PATH:-/home/ws/install/my_robot_controller/share/my_robot_controller/tracks/track_5_figure_eight.csv}"
COMMON_GAZEBO_SEED="${PERTURBATION_GAZEBO_SEED:-44}"
COMMON_NOISE_SEED="${PERTURBATION_NOISE_SEED:-2026}"

case "${CONTROLLER_FAMILY}" in
  pid)
    DEFAULT_CONFIG="/home/ws/install/my_robot_controller/share/my_robot_controller/config/pid_cascade.yaml"
    ;;
  lqr)
    DEFAULT_CONFIG="/home/ws/install/my_robot_controller/share/my_robot_controller/config/lqr.yaml"
    ;;
  mpc)
    DEFAULT_CONFIG="/home/ws/install/my_robot_controller/share/my_robot_controller/config/mpc.yaml"
    ;;
  *)
    echo "Unsupported PERTURBATION_CONTROLLER_FAMILY: ${CONTROLLER_FAMILY}"
    exit 2
    ;;
esac
COMMON_CONFIG="${PERTURBATION_CONFIG_PATH:-${DEFAULT_CONFIG}}"

mkdir -p "${RESULT_DIR}"
failed_cases=0

run_case()
{
  local scenario="$1"
  local domain="none"
  local command_enabled="false"
  local feedback_enabled="false"
  local start="5.0"
  local start_delays=""
  local duration="1.0"
  local persistent="false"
  local angular_bias="0.0"
  local left_effectiveness="1.0"
  local right_effectiveness="1.0"
  local command_delay="0.0"
  local position_noise="0.0"
  local yaw_noise="0.0"
  local odometry_x_bias="0.0"
  local odometry_y_bias="0.0"
  local odometry_yaw_bias="0.0"

  case "${scenario}" in
    nominal)
      ;;
    angular_pulse|angular_pulse_train)
      domain="additive_actuator_fault"
      command_enabled="true"
      start="6.0"
      start_delays="6.0;18.0;30.0"
      duration="1.0"
      angular_bias="0.8"
      ;;
    angular_constant)
      domain="persistent_actuator_bias"
      command_enabled="true"
      duration="15.0"
      angular_bias="0.2"
      ;;
    left_wheel_loss)
      domain="multiplicative_actuator_fault"
      command_enabled="true"
      duration="2.0"
      left_effectiveness="0.7"
      ;;
    left_wheel_loss_persistent)
      domain="persistent_multiplicative_actuator_fault"
      command_enabled="true"
      persistent="true"
      left_effectiveness="0.7"
      ;;
    command_delay)
      domain="transport_delay"
      command_enabled="true"
      duration="5.0"
      command_delay="0.10"
      ;;
    localization_noise)
      domain="measurement_path_noise"
      feedback_enabled="true"
      duration="5.0"
      position_noise="0.01"
      yaw_noise="0.02"
      ;;
    localization_yaw_bias)
      domain="measurement_path_bias"
      feedback_enabled="true"
      duration="10.0"
      odometry_yaw_bias="0.08"
      ;;
    *)
      echo "Unknown perturbation scenario: ${scenario}"
      return 2
      ;;
  esac

  echo "Running scenario: ${scenario} (${domain})"
  if ! PERTURBATION_SCENARIO="${scenario}" \
    PERTURBATION_DOMAIN="${domain}" \
    PERTURBATION_GUI="${COMMON_GUI}" \
    PERTURBATION_CONTROLLER_FAMILY="${CONTROLLER_FAMILY}" \
    PERTURBATION_TRACK_PATH="${COMMON_TRACK}" \
    PERTURBATION_CONFIG_PATH="${COMMON_CONFIG}" \
    PERTURBATION_GAZEBO_SEED="${COMMON_GAZEBO_SEED}" \
    COMMAND_FAULT_ENABLED="${command_enabled}" \
    FEEDBACK_FAULT_ENABLED="${feedback_enabled}" \
    FAULT_START_DELAY="${start}" \
    FAULT_START_DELAYS="${start_delays}" \
    FAULT_DURATION="${duration}" \
    FAULT_PERSISTENT="${persistent}" \
    ANGULAR_VELOCITY_BIAS="${angular_bias}" \
    LEFT_WHEEL_EFFECTIVENESS="${left_effectiveness}" \
    RIGHT_WHEEL_EFFECTIVENESS="${right_effectiveness}" \
    COMMAND_DELAY="${command_delay}" \
    POSITION_NOISE_STDDEV="${position_noise}" \
    YAW_NOISE_STDDEV="${yaw_noise}" \
    ODOMETRY_X_BIAS="${odometry_x_bias}" \
    ODOMETRY_Y_BIAS="${odometry_y_bias}" \
    ODOMETRY_YAW_BIAS="${odometry_yaw_bias}" \
    PERTURBATION_NOISE_SEED="${COMMON_NOISE_SEED}" \
    bash scripts/run_pid_perturbation_case.sh "${RESULT_DIR}/${scenario}"
  then
    failed_cases=$((failed_cases + 1))
  fi
}

for scenario in ${SCENARIOS}; do
  run_case "${scenario}"
done

if [[ -f "${RESULT_DIR}/nominal/ground_truth.csv" ]]; then
  python3 scripts/analyze_perturbation_suite.py "${RESULT_DIR}"
else
  echo "Nominal baseline is missing; aggregate analysis cannot be generated"
  failed_cases=$((failed_cases + 1))
fi

if [[ "${failed_cases}" -ne 0 ]]; then
  echo "Perturbation suite finished with ${failed_cases} failed case(s)"
  exit 1
fi

echo "Perturbation suite complete: ${RESULT_DIR}/summary.csv"

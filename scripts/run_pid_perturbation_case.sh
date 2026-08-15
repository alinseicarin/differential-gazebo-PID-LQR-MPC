#!/usr/bin/env bash
# Execute one PID, LQR, or MPC robustness scenario in a fresh Gazebo instance.
#
# This is the low-level runner used by run_pid_perturbation_suite.sh. Scenario
# parameters are supplied through environment variables. The same mechanism
# wraps PID, TVLQR, and MPC without changing the injected faults.

set -eo pipefail

cd /home/ws
source /opt/ros/humble/setup.bash
source /home/ws/install/setup.bash

set -u
export ROS2CLI_DISABLE_DAEMON=1

RESULT_DIR="${1:?usage: run_pid_perturbation_case.sh RESULT_DIR}"
SCENARIO_NAME="${PERTURBATION_SCENARIO:-nominal}"
FAULT_DOMAIN="${PERTURBATION_DOMAIN:-none}"
CONTROLLER_FAMILY="${PERTURBATION_CONTROLLER_FAMILY:-pid}"
GUI="${PERTURBATION_GUI:-false}"
GAZEBO_SEED="${PERTURBATION_GAZEBO_SEED:-42}"
TIMEOUT_SECONDS="${PERTURBATION_TIMEOUT_SECONDS:-75}"
MPC_HORIZON_STEPS="${MPC_PREDICTION_HORIZON_STEPS:-}"
CONTROLLER_EXTRA_ARGS=()

case "${CONTROLLER_FAMILY}" in
  pid)
    DEFAULT_CONFIG_PATH="/home/ws/install/my_robot_controller/share/my_robot_controller/config/pid_cascade.yaml"
    CONTROLLER_LAUNCH="pid_perturbation_suite.launch.py"
    ;;
  lqr)
    DEFAULT_CONFIG_PATH="/home/ws/install/my_robot_controller/share/my_robot_controller/config/lqr.yaml"
    CONTROLLER_LAUNCH="lqr_perturbation_suite.launch.py"
    ;;
  mpc)
    MPC_HORIZON_STEPS="${MPC_HORIZON_STEPS:-45}"
    DEFAULT_CONFIG_PATH="/home/ws/install/my_robot_controller/share/my_robot_controller/config/mpc.yaml"
    CONTROLLER_LAUNCH="mpc_perturbation_suite.launch.py"
    CONTROLLER_EXTRA_ARGS+=(
      prediction_horizon_steps:="${MPC_HORIZON_STEPS}"
    )
    ;;
  *)
    echo "Unsupported PERTURBATION_CONTROLLER_FAMILY: ${CONTROLLER_FAMILY}"
    exit 2
    ;;
esac

CONFIG_PATH="${PERTURBATION_CONFIG_PATH:-${DEFAULT_CONFIG_PATH}}"
REFERENCE_CONFIG_PATH="${PERTURBATION_REFERENCE_CONFIG_PATH:-/home/ws/install/my_robot_controller/share/my_robot_controller/config/trajectory_reference.yaml}"
TRACK_SOURCE="${PERTURBATION_TRACK_PATH:-/home/ws/install/my_robot_controller/share/my_robot_controller/tracks/track_5_figure_eight.csv}"
TRACK_PATH="${RESULT_DIR}/track_under_test.csv"

COMMAND_FAULT_ENABLED="${COMMAND_FAULT_ENABLED:-false}"
FEEDBACK_FAULT_ENABLED="${FEEDBACK_FAULT_ENABLED:-false}"
FAULT_START_DELAY="${FAULT_START_DELAY:-5.0}"
FAULT_START_DELAYS="${FAULT_START_DELAYS:-}"
FAULT_DURATION="${FAULT_DURATION:-1.0}"
FAULT_PERSISTENT="${FAULT_PERSISTENT:-false}"
LINEAR_VELOCITY_BIAS="${LINEAR_VELOCITY_BIAS:-0.0}"
ANGULAR_VELOCITY_BIAS="${ANGULAR_VELOCITY_BIAS:-0.0}"
LEFT_WHEEL_EFFECTIVENESS="${LEFT_WHEEL_EFFECTIVENESS:-1.0}"
RIGHT_WHEEL_EFFECTIVENESS="${RIGHT_WHEEL_EFFECTIVENESS:-1.0}"
COMMAND_DELAY="${COMMAND_DELAY:-0.0}"
ODOMETRY_X_BIAS="${ODOMETRY_X_BIAS:-0.0}"
ODOMETRY_Y_BIAS="${ODOMETRY_Y_BIAS:-0.0}"
ODOMETRY_YAW_BIAS="${ODOMETRY_YAW_BIAS:-0.0}"
POSITION_NOISE_STDDEV="${POSITION_NOISE_STDDEV:-0.0}"
YAW_NOISE_STDDEV="${YAW_NOISE_STDDEV:-0.0}"
NOISE_SEED="${PERTURBATION_NOISE_SEED:-2026}"

# ROS 2 launch treats `name:=` as a malformed command-line argument.  Keep the
# repeated-window parameter optional so legacy single-window scenarios can use
# the node's empty-string default without emitting an invalid launch token.
FAULT_SCHEDULE_ARGS=()
if [[ -n "${FAULT_START_DELAYS}" ]]; then
  FAULT_SCHEDULE_ARGS+=(fault_start_delays:="${FAULT_START_DELAYS}")
fi

CONTROLLER_CSV="${RESULT_DIR}/controller.csv"
COMMAND_CSV="${RESULT_DIR}/applied_commands.csv"
ODOMETRY_CSV="${RESULT_DIR}/disturbed_odometry.csv"
GROUND_TRUTH_CSV="${RESULT_DIR}/ground_truth.csv"
SIM_LOG="${RESULT_DIR}/simulation.log"
CONTROL_LOG="${RESULT_DIR}/control_graph.log"
METADATA_CSV="${RESULT_DIR}/scenario_metadata.csv"

mkdir -p "${RESULT_DIR}"
if [[ ! -f "${TRACK_SOURCE}" ]]; then
  echo "Track does not exist: ${TRACK_SOURCE}"
  exit 2
fi
if [[ ! -f "${CONFIG_PATH}" || ! -f "${REFERENCE_CONFIG_PATH}" ]]; then
  echo "Controller or reference configuration does not exist"
  exit 2
fi
cp "${TRACK_SOURCE}" "${TRACK_PATH}"
CONFIG_SHA256="$(sha256sum "${CONFIG_PATH}" | awk '{print $1}')"
REFERENCE_CONFIG_SHA256="$(sha256sum "${REFERENCE_CONFIG_PATH}" | awk '{print $1}')"
TRACK_SHA256="$(sha256sum "${TRACK_PATH}" | awk '{print $1}')"

ACTIVE_SIMULATION_PID=""
ACTIVE_CONTROL_PID=""

stop_process_group()
{
  local process_id="$1"
  if [[ -z "${process_id}" ]]; then
    return
  fi
  kill -INT -- "-${process_id}" 2>/dev/null || true
  sleep 1
  kill -TERM -- "-${process_id}" 2>/dev/null || true
  kill -TERM "${process_id}" 2>/dev/null || true
  wait "${process_id}" 2>/dev/null || true
}

cleanup()
{
  stop_process_group "${ACTIVE_CONTROL_PID}"
  ACTIVE_CONTROL_PID=""
  stop_process_group "${ACTIVE_SIMULATION_PID}"
  ACTIVE_SIMULATION_PID=""
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "[${SCENARIO_NAME}] starting Gazebo (gui=${GUI}, seed=${GAZEBO_SEED})"
setsid ros2 launch my_robot_description display.launch.py \
  world:=empty.world gui:="${GUI}" seed:="${GAZEBO_SEED}" > "${SIM_LOG}" 2>&1 &
ACTIVE_SIMULATION_PID=$!

simulation_ready=0
for second in $(seq 1 45); do
  if grep -q 'Successfully spawned entity' "${SIM_LOG}" 2>/dev/null; then
    simulation_ready=1
    break
  fi
  if ! kill -0 "${ACTIVE_SIMULATION_PID}" 2>/dev/null; then
    break
  fi
  sleep 1
done
if [[ "${simulation_ready}" -ne 1 ]]; then
  echo "[${SCENARIO_NAME}] Gazebo failed to spawn the robot"
  tail -n 40 "${SIM_LOG}" || true
  exit 1
fi

sleep 1
setsid ros2 launch my_robot_controller "${CONTROLLER_LAUNCH}" \
  config_path:="${CONFIG_PATH}" \
  reference_config_path:="${REFERENCE_CONFIG_PATH}" \
  csv_path:="${TRACK_PATH}" \
  controller_output_csv_path:="${CONTROLLER_CSV}" \
  applied_command_csv_path:="${COMMAND_CSV}" \
  disturbed_odometry_csv_path:="${ODOMETRY_CSV}" \
  evaluation_output_csv_path:="${GROUND_TRUTH_CSV}" \
  command_fault_enabled:="${COMMAND_FAULT_ENABLED}" \
  feedback_fault_enabled:="${FEEDBACK_FAULT_ENABLED}" \
  fault_start_delay:="${FAULT_START_DELAY}" \
  "${FAULT_SCHEDULE_ARGS[@]}" \
  fault_duration:="${FAULT_DURATION}" \
  fault_persistent:="${FAULT_PERSISTENT}" \
  linear_velocity_bias:="${LINEAR_VELOCITY_BIAS}" \
  angular_velocity_bias:="${ANGULAR_VELOCITY_BIAS}" \
  left_wheel_effectiveness:="${LEFT_WHEEL_EFFECTIVENESS}" \
  right_wheel_effectiveness:="${RIGHT_WHEEL_EFFECTIVENESS}" \
  wheel_separation:=0.35 \
  command_delay:="${COMMAND_DELAY}" \
  odometry_x_bias:="${ODOMETRY_X_BIAS}" \
  odometry_y_bias:="${ODOMETRY_Y_BIAS}" \
  odometry_yaw_bias:="${ODOMETRY_YAW_BIAS}" \
  position_noise_stddev:="${POSITION_NOISE_STDDEV}" \
  yaw_noise_stddev:="${YAW_NOISE_STDDEV}" \
  noise_seed:="${NOISE_SEED}" \
  "${CONTROLLER_EXTRA_ARGS[@]}" > "${CONTROL_LOG}" 2>&1 &
ACTIVE_CONTROL_PID=$!

complete=0
graph_failed=0
for second in $(seq 1 "${TIMEOUT_SECONDS}"); do
  if grep -q 'Trajectory complete' "${CONTROL_LOG}" 2>/dev/null; then
    complete=1
    break
  fi
  if ! kill -0 "${ACTIVE_CONTROL_PID}" 2>/dev/null; then
    graph_failed=1
    break
  fi
  if ((second % 15 == 0)); then
    echo "[${SCENARIO_NAME}] waiting: ${second}s"
  fi
  sleep 1
done

# Preserve two seconds of post-completion ground truth so endpoint motion and
# settling remain visible in the dataset. All predefined faults end earlier.
if [[ "${complete}" -eq 1 ]]; then
  sleep 2
fi

stop_process_group "${ACTIVE_CONTROL_PID}"
ACTIVE_CONTROL_PID=""
stop_process_group "${ACTIVE_SIMULATION_PID}"
ACTIVE_SIMULATION_PID=""

if [[ "${graph_failed}" -eq 1 ]]; then
  echo "[${SCENARIO_NAME}] control graph exited unexpectedly"
  tail -n 60 "${CONTROL_LOG}" || true
  exit 1
fi
if [[ ! -s "${CONTROLLER_CSV}" || ! -s "${COMMAND_CSV}" ||
  ! -s "${ODOMETRY_CSV}" || ! -s "${GROUND_TRUTH_CSV}" ]]
then
  echo "[${SCENARIO_NAME}] one or more experiment CSV files are missing"
  tail -n 60 "${CONTROL_LOG}" || true
  exit 1
fi

{
  echo "scenario,fault_domain,fault_start,fault_duration,track_complete,command_fault_enabled,feedback_fault_enabled,angular_bias,left_wheel_effectiveness,right_wheel_effectiveness,command_delay,position_noise_stddev,yaw_noise_stddev,noise_seed,gazebo_seed,controller_family,mpc_horizon_steps,controller_config_sha256,reference_config_sha256,track_sha256,fault_start_delays,fault_persistent,odometry_x_bias,odometry_y_bias,odometry_yaw_bias"
  echo "${SCENARIO_NAME},${FAULT_DOMAIN},${FAULT_START_DELAY},${FAULT_DURATION},${complete},${COMMAND_FAULT_ENABLED},${FEEDBACK_FAULT_ENABLED},${ANGULAR_VELOCITY_BIAS},${LEFT_WHEEL_EFFECTIVENESS},${RIGHT_WHEEL_EFFECTIVENESS},${COMMAND_DELAY},${POSITION_NOISE_STDDEV},${YAW_NOISE_STDDEV},${NOISE_SEED},${GAZEBO_SEED},${CONTROLLER_FAMILY},${MPC_HORIZON_STEPS},${CONFIG_SHA256},${REFERENCE_CONFIG_SHA256},${TRACK_SHA256},${FAULT_START_DELAYS},${FAULT_PERSISTENT},${ODOMETRY_X_BIAS},${ODOMETRY_Y_BIAS},${ODOMETRY_YAW_BIAS}"
} > "${METADATA_CSV}"

if [[ "${complete}" -eq 1 ]]; then
  echo "[${SCENARIO_NAME}] complete"
else
  echo "[${SCENARIO_NAME}] did not complete within ${TIMEOUT_SECONDS}s"
fi
grep -E 'fault (started|ended)|Trajectory complete' "${CONTROL_LOG}" | tail -n 8 || true

#!/usr/bin/env bash
# Run the cascaded PID through a reproducible command-path yaw fault.
#
# The PID publishes /cmd_vel_nominal. A separate injector adds a simulation-time
# yaw-rate bias for a fixed window and publishes /cmd_vel to Gazebo. No physical
# force is applied, so wheel loading, friction, and caster contact are unchanged.
#
# Usage inside the development container:
#   bash scripts/run_cascade_command_disturbance.sh [result_directory]
# To use a complete alternative benchmark track:
#   DISTURBANCE_TRACK_PATH=/path/to/track_5_figure_eight.csv \
#     bash scripts/run_cascade_command_disturbance.sh [result_directory]
#
# Set DISTURBANCE_GUI=false for an automated commissioning run. The default is
# true so the final test is visible in Gazebo and RViz.

set -eo pipefail

cd /home/ws
source /opt/ros/humble/setup.bash
source /home/ws/install/setup.bash

set -u
export ROS2CLI_DISABLE_DAEMON=1

RESULT_DIR="${1:-/home/ws/results/cascade_command_disturbance}"
GUI="${DISTURBANCE_GUI:-true}"
FAULT_START_DELAY="${FAULT_START_DELAY:-5.0}"
FAULT_DURATION="${FAULT_DURATION:-1.0}"
ANGULAR_VELOCITY_BIAS="${ANGULAR_VELOCITY_BIAS:-0.6}"
RECOVERY_CTE_THRESHOLD="${RECOVERY_CTE_THRESHOLD:-0.010}"
RECOVERY_HEADING_THRESHOLD="${RECOVERY_HEADING_THRESHOLD:-0.01}"
GAZEBO_SEED="${DISTURBANCE_GAZEBO_SEED:-42}"
SETTLING_SIM_TIME="${DISTURBANCE_SETTLING_TIME:-2.0}"
SETTLING_LINEAR_THRESHOLD="${DISTURBANCE_SETTLING_LINEAR_THRESHOLD:-0.01}"
SETTLING_ANGULAR_THRESHOLD="${DISTURBANCE_SETTLING_ANGULAR_THRESHOLD:-0.02}"

CONFIG_PATH="${DISTURBANCE_CONFIG_PATH:-/home/ws/install/my_robot_controller/share/my_robot_controller/config/pid_cascade.yaml}"
INSTALLED_STRAIGHT="/home/ws/install/my_robot_controller/share/my_robot_controller/tracks/track_1_straight.csv"
TRACK_SOURCE="${DISTURBANCE_TRACK_PATH:-${INSTALLED_STRAIGHT}}"
TRACK_PATH="${RESULT_DIR}/track_under_test.csv"
TRACK_POINT_COUNT="${DISTURBANCE_TRACK_POINTS:-}"
CONTROLLER_CSV="${RESULT_DIR}/cascade_controller.csv"
APPLIED_COMMAND_CSV="${RESULT_DIR}/applied_commands.csv"
GROUND_TRUTH_CSV="${RESULT_DIR}/ground_truth_trajectory.csv"
SUMMARY_PATH="${RESULT_DIR}/recovery_summary.txt"
SIM_LOG="${RESULT_DIR}/simulation.log"
CONTROL_GRAPH_LOG="${RESULT_DIR}/controller_and_injector.log"

mkdir -p "${RESULT_DIR}"
if [[ ! -f "${TRACK_SOURCE}" ]]; then
  echo "Disturbance track does not exist: ${TRACK_SOURCE}"
  exit 2
fi

if [[ -n "${TRACK_POINT_COUNT}" ]]; then
  if ! [[ "${TRACK_POINT_COUNT}" =~ ^[0-9]+$ ]] ||
    [[ "${TRACK_POINT_COUNT}" -lt 2 ]]
  then
    echo "DISTURBANCE_TRACK_POINTS must be an integer greater than one"
    exit 2
  fi
  # An explicit point count supports shortened or extended straight-line
  # diagnostics while retaining the same source file.
  head -n "${TRACK_POINT_COUNT}" "${TRACK_SOURCE}" > "${TRACK_PATH}"
elif [[ "${TRACK_SOURCE}" == "${INSTALLED_STRAIGHT}" ]]; then
  # Preserve the historical default: the first 101 points form a 5 m line.
  head -n 101 "${TRACK_SOURCE}" > "${TRACK_PATH}"
else
  # Alternative tracks such as the circle or figure eight are copied in full;
  # silently truncating them would change the requested benchmark geometry.
  cp "${TRACK_SOURCE}" "${TRACK_PATH}"
fi

ACTIVE_SIMULATION_PID=""
ACTIVE_CONTROL_GRAPH_PID=""

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
  stop_process_group "${ACTIVE_CONTROL_GRAPH_PID}"
  ACTIVE_CONTROL_GRAPH_PID=""
  stop_process_group "${ACTIVE_SIMULATION_PID}"
  ACTIVE_SIMULATION_PID=""
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "Starting Gazebo command-disturbance test (gui=${GUI})"
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
  echo "Gazebo failed to spawn the robot"
  tail -n 40 "${SIM_LOG}" || true
  exit 1
fi

sleep 1
setsid ros2 launch my_robot_controller pid_command_disturbance.launch.py \
  config_path:="${CONFIG_PATH}" \
  csv_path:="${TRACK_PATH}" \
  controller_output_csv_path:="${CONTROLLER_CSV}" \
  applied_command_csv_path:="${APPLIED_COMMAND_CSV}" \
  evaluation_output_csv_path:="${GROUND_TRUTH_CSV}" \
  fault_start_delay:="${FAULT_START_DELAY}" \
  fault_duration:="${FAULT_DURATION}" \
  angular_velocity_bias:="${ANGULAR_VELOCITY_BIAS}" \
  linear_velocity_bias:=0.0 > "${CONTROL_GRAPH_LOG}" 2>&1 &
ACTIVE_CONTROL_GRAPH_PID=$!

echo "Fault schedule: start=${FAULT_START_DELAY}s duration=${FAULT_DURATION}s angular_bias=${ANGULAR_VELOCITY_BIAS}rad/s"

complete=0
for second in $(seq 1 60); do
  if grep -q 'Trajectory complete' "${CONTROL_GRAPH_LOG}" 2>/dev/null; then
    complete=1
    break
  fi
  if ! kill -0 "${ACTIVE_CONTROL_GRAPH_PID}" 2>/dev/null; then
    break
  fi
  if ((second % 10 == 0)); then
    echo "Waiting for track completion: ${second}s"
  fi
  sleep 1
done

completion_stamp=""
settled=0
if [[ "${complete}" -eq 1 ]]; then
  completion_stamp="$(sed -n \
    's/.*Trajectory complete at simulation time \([0-9][0-9.]*\) s.*/\1/p' \
    "${CONTROL_GRAPH_LOG}" | tail -n 1)"
fi

if [[ -n "${completion_stamp}" ]]; then
  settling_target="$(awk -v stamp="${completion_stamp}" \
    -v duration="${SETTLING_SIM_TIME}" 'BEGIN {printf "%.9f", stamp + duration}')"
  for attempt in $(seq 1 150); do
    if [[ -s "${GROUND_TRUTH_CSV}" ]]; then
      IFS=',' read -r truth_stamp truth_linear_velocity truth_angular_velocity < <(
        tail -n 1 "${GROUND_TRUTH_CSV}" | awk -F, '{print $2 "," $33 "," $34}')
      if awk \
        -v stamp="${truth_stamp}" -v target="${settling_target}" \
        -v linear="${truth_linear_velocity}" -v angular="${truth_angular_velocity}" \
        -v linear_limit="${SETTLING_LINEAR_THRESHOLD}" \
        -v angular_limit="${SETTLING_ANGULAR_THRESHOLD}" '
        function abs(value) {return value < 0 ? -value : value}
        BEGIN {
          exit !(stamp >= target && abs(linear) <= linear_limit &&
            abs(angular) <= angular_limit)
        }'
      then
        settled=1
        break
      fi
    fi
    sleep 0.1
  done
fi

if [[ "${complete}" -eq 1 && "${settled}" -ne 1 ]]; then
  echo "Robot did not satisfy the fixed terminal settling condition"
  complete=0
fi

stop_process_group "${ACTIVE_CONTROL_GRAPH_PID}"
ACTIVE_CONTROL_GRAPH_PID=""
stop_process_group "${ACTIVE_SIMULATION_PID}"
ACTIVE_SIMULATION_PID=""

if [[ ! -s "${CONTROLLER_CSV}" || ! -s "${APPLIED_COMMAND_CSV}" ||
  ! -s "${GROUND_TRUTH_CSV}" ]]
then
  echo "Controller, applied-command, or ground-truth CSV is missing"
  tail -n 40 "${CONTROL_GRAPH_LOG}" || true
  exit 1
fi

fault_start="$(awk -F, 'NR > 1 && $6 == 1 {print $1; exit}' "${APPLIED_COMMAND_CSV}")"
fault_start_stamp="$(awk -F, 'NR > 1 && $6 == 1 {print $7; exit}' "${APPLIED_COMMAND_CSV}")"
fault_end="$(awk -F, '
  NR > 1 {
    if (previous_active == 1 && $6 == 0) {print $1; exit}
    previous_active = $6
  }
' "${APPLIED_COMMAND_CSV}")"
fault_end_stamp="$(awk -F, '
  NR > 1 {
    if (previous_active == 1 && $6 == 0) {print $7; exit}
    previous_active = $6
  }
' "${APPLIED_COMMAND_CSV}")"

if [[ -z "${fault_start}" || -z "${fault_end}" ||
  -z "${fault_start_stamp}" || -z "${fault_end_stamp}" ]]
then
  echo "The applied-command log does not contain a complete fault window"
  exit 1
fi

# Recovery requires ten consecutive ground-truth samples with cross-track error
# below 1 cm and path-heading error below 0.01 rad by default. The 1 cm spatial
# band is fixed at roughly twice the largest terminal CTE measured in the
# repeated nominal straight-line commissioning runs; the former 5 mm band was
# narrower than the nominal ground-truth/localization residual in two of three
# runs and could falsely label an otherwise settled response as unrecovered.
# Absolute Gazebo timestamps align evaluation with the downstream fault window.
awk -F, \
  -v fault_start="${fault_start}" -v fault_end="${fault_end}" \
  -v fault_start_stamp="${fault_start_stamp}" -v fault_end_stamp="${fault_end_stamp}" \
  -v complete="${complete}" \
  -v cte_threshold="${RECOVERY_CTE_THRESHOLD}" \
  -v heading_threshold="${RECOVERY_HEADING_THRESHOLD}" '
  NR == 1 {next}
  {
    time = $1
    stamp = $2
    cte = $18
    heading = $19
    abs_cte = cte < 0 ? -cte : cte
    abs_heading = heading < 0 ? -heading : heading

    if (stamp >= fault_start_stamp) {
      if (abs_cte > peak_cte) {
        peak_cte = abs_cte
        peak_cte_stamp = stamp
        stable_count = 0
        recovery_time = ""
      }
      if (abs_heading > peak_heading) peak_heading = abs_heading

      if (stamp >= fault_end_stamp && stamp >= peak_cte_stamp &&
        abs_cte <= cte_threshold && abs_heading <= heading_threshold)
      {
        if (stable_count == 0) stable_start_stamp = stamp
        stable_count++
        if (stable_count >= 10 && recovery_time == "") {
          recovery_time = stable_start_stamp - fault_end_stamp
        }
      } else {
        stable_count = 0
      }
    }

    final_time = time
    final_cte = abs_cte
    final_x = $3
    final_y = $4
  }
  END {
    printf "COMMAND_DISTURBANCE_RESULT\n"
    printf "track_complete=%d\n", complete
    printf "fault_start=%.3f s\n", fault_start
    printf "fault_end=%.3f s\n", fault_end
    printf "measured_fault_duration=%.3f s\n", fault_end - fault_start
    printf "peak_abs_cross_track_error=%.6f m\n", peak_cte
    printf "peak_abs_path_heading_error=%.6f rad\n", peak_heading
    if (recovery_time == "") printf "recovery_after_fault=not_recovered\n"
    else printf "recovery_after_fault=%.3f s\n", recovery_time
    printf "final_abs_cross_track_error=%.6f m\n", final_cte
    printf "final_position=(%.6f, %.6f) m\n", final_x, final_y
    printf "experiment_duration=%.3f s\n", final_time
  }
' "${GROUND_TRUTH_CSV}" | tee "${SUMMARY_PATH}"

awk -F, '
  NR == 1 {next}
  {
    difference = $5 - $3
    abs_difference = difference < 0 ? -difference : difference
    abs_applied = $5 < 0 ? -$5 : $5
    if ($6 == 1) {
      active_samples++
      if (abs_difference > max_bias) max_bias = abs_difference
    }
    if (abs_applied > max_applied) max_applied = abs_applied
  }
  END {
    printf "fault_active_samples=%d\n", active_samples
    printf "measured_max_abs_angular_bias=%.6f rad/s\n", max_bias
    printf "max_abs_applied_angular_command=%.6f rad/s\n", max_applied
  }
' "${APPLIED_COMMAND_CSV}" | tee -a "${SUMMARY_PATH}"

grep -E 'Command fault (started|ended)|Trajectory complete' \
  "${CONTROL_GRAPH_LOG}" | tail -n 5 || true

if [[ "${complete}" -ne 1 ]]; then
  echo "The robot did not complete the path within the timeout"
  tail -n 40 "${CONTROL_GRAPH_LOG}" || true
  exit 1
fi

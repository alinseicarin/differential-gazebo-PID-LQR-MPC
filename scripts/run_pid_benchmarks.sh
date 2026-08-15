#!/usr/bin/env bash
# Run named PID profiles on five time-parameterized reference trajectories.
#
# Usage from inside the development container:
#   bash scripts/run_pid_benchmarks.sh [result_directory]
#
# Optional selectors can reduce a development run without changing the default
# cascaded-PID, five-track, three-repetition formal matrix. For example:
#   PID_BENCHMARK_GUI=true \
#   PID_BENCHMARK_TRACKS="circle figure_eight" \
#   PID_BENCHMARK_REPETITIONS=1 \
#     bash scripts/run_pid_benchmarks.sh /tmp/pid_closed_paths
#
# Legacy lookahead profiles require an explicit PID_BENCHMARK_PROFILES override.
#
# The script launches a fresh headless Gazebo instance for every trial so robot
# state, EKF state, and simulation time cannot leak between configurations.

set -eo pipefail

cd /home/ws
source /opt/ros/humble/setup.bash
source /home/ws/install/setup.bash

# Enable undefined-variable checking only after ROS setup scripts have finished;
# those scripts legitimately inspect optional environment variables.
set -u
export ROS2CLI_DISABLE_DAEMON=1

RESULT_DIR="${1:-/tmp/pid_benchmarks}"
mkdir -p "${RESULT_DIR}"
GUI="${PID_BENCHMARK_GUI:-false}"
REPETITIONS="${PID_BENCHMARK_REPETITIONS:-3}"
BASE_SEED="${PID_BENCHMARK_BASE_SEED:-42}"
SETTLING_SIM_TIME="${PID_BENCHMARK_SETTLING_TIME:-2.0}"
SETTLING_LINEAR_THRESHOLD="${PID_BENCHMARK_SETTLING_LINEAR_THRESHOLD:-0.01}"
SETTLING_ANGULAR_THRESHOLD="${PID_BENCHMARK_SETTLING_ANGULAR_THRESHOLD:-0.02}"
SETTLING_WALL_TIMEOUT="${PID_BENCHMARK_SETTLING_WALL_TIMEOUT:-15}"

if [[ "${GUI}" != "true" && "${GUI}" != "false" ]] ||
  ! [[ "${REPETITIONS}" =~ ^[1-9][0-9]*$ ]] ||
  ! [[ "${BASE_SEED}" =~ ^[0-9]+$ ]] ||
  ! [[ "${SETTLING_WALL_TIMEOUT}" =~ ^[1-9][0-9]*$ ]]
then
  echo "GUI must be true/false; repetitions and settling timeout must be positive; base seed must be non-negative" >&2
  exit 2
fi

# Use the first 101 points of the 20 m line, producing a 5 m
# straight trial comparable in scale with the 5 m sinusoidal path.
STRAIGHT_TRACK="${RESULT_DIR}/track_straight_5m.csv"
head -n 101 \
  /home/ws/install/my_robot_controller/share/my_robot_controller/tracks/track_1_straight.csv \
  > "${STRAIGHT_TRACK}"

CURVE_TRACK="/home/ws/install/my_robot_controller/share/my_robot_controller/tracks/track_2_curve.csv"
CORNER_TRACK="/home/ws/install/my_robot_controller/share/my_robot_controller/tracks/track_3_corner.csv"
CIRCLE_TRACK="/home/ws/install/my_robot_controller/share/my_robot_controller/tracks/track_4_circle.csv"
FIGURE_EIGHT_TRACK="/home/ws/install/my_robot_controller/share/my_robot_controller/tracks/track_5_figure_eight.csv"
INITIAL_LATERAL_TRACK="/home/ws/install/my_robot_controller/share/my_robot_controller/tracks/track_6_initial_lateral_offset.csv"
INITIAL_HEADING_TRACK="/home/ws/install/my_robot_controller/share/my_robot_controller/tracks/track_7_initial_heading_offset.csv"
ESTIMATOR_SUMMARY="${RESULT_DIR}/estimated_state_summary.csv"
TRUTH_SUMMARY="${RESULT_DIR}/ground_truth_summary.csv"
# The controller retains its strict 8 cm estimator-frame stopping rule. Truth
# success permits half the 0.30 m chassis length because wheel/IMU odometry has
# no absolute world-position correction; the exact true error is always logged.
TRUTH_GOAL_TOLERANCE="${TRUTH_GOAL_TOLERANCE:-0.15}"
ACTIVE_SIMULATION_PID=""
ACTIVE_CONTROLLER_PID=""

printf '%s\n' \
  'profile,track,repetition,seed,success,control_duration,estimated_final_error,estimated_position_rmse,estimated_longitudinal_rmse,estimated_lateral_rmse,estimated_trajectory_heading_rmse,estimated_cte_rmse,estimated_cte_max,estimated_path_heading_rmse,estimated_path_heading_max,normalized_command_activity,max_v,max_w,final_estimated_x,final_estimated_y,samples' \
  > "${ESTIMATOR_SUMMARY}"
printf '%s\n' \
  'profile,track,repetition,seed,success,control_duration,true_final_error,true_position_rmse,true_longitudinal_rmse,true_lateral_rmse,true_trajectory_heading_rmse,true_cte_rmse,true_cte_max,true_path_heading_rmse,true_path_heading_max,localization_rmse,localization_max,final_truth_x,final_truth_y,samples' \
  > "${TRUTH_SUMMARY}"

# Stop an entire process group created with setsid. ROS launch normally cleans
# up its children on SIGINT; SIGTERM is a fallback for a stuck child process.
stop_process_group()
{
  local process_id="$1"
  if [[ -z "${process_id}" ]]; then
    return
  fi
  kill -INT -- "-${process_id}" 2>/dev/null || true
  sleep 1
  kill -TERM -- "-${process_id}" 2>/dev/null || true
  wait "${process_id}" 2>/dev/null || true
}

# An interrupted shell script does not automatically stop the independent
# process groups created by setsid. Track the currently active groups so Ctrl-C,
# SIGTERM, and ordinary script exit cannot leave Gazebo or a controller behind.
cleanup_active_processes()
{
  stop_process_group "${ACTIVE_CONTROLLER_PID}"
  ACTIVE_CONTROLLER_PID=""
  stop_process_group "${ACTIVE_SIMULATION_PID}"
  ACTIVE_SIMULATION_PID=""
}

trap cleanup_active_processes EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

run_trial()
{
  local profile="$1"
  local track_name="$2"
  local track_path="$3"
  local repetition="$4"
  local seed="$5"
  local world_name="${6:-empty.world}"
  local trial="${profile}_${track_name}_run${repetition}"
  local sim_log="${RESULT_DIR}/${trial}_sim.log"
  local controller_log="${RESULT_DIR}/${trial}_controller.log"
  local output_csv="${RESULT_DIR}/${trial}.csv"
  local truth_csv="${RESULT_DIR}/${trial}_ground_truth.csv"
  local config_path
  config_path="/home/ws/install/my_robot_controller/share/my_robot_controller/config/pid_${profile}.yaml"

  echo "START ${trial}"

  # setsid gives each launch graph its own process group for reliable cleanup.
  setsid ros2 launch my_robot_description display.launch.py \
    world:="${world_name}" gui:="${GUI}" seed:="${seed}" > "${sim_log}" 2>&1 &
  local simulation_pid=$!
  ACTIVE_SIMULATION_PID="${simulation_pid}"

  # Wait for the entity rather than sleeping a fixed time. This accommodates
  # different host loads without letting the robot drift unnecessarily.
  local simulation_ready=0
  local second
  for second in $(seq 1 45); do
    if grep -q 'Successfully spawned entity' "${sim_log}" 2>/dev/null; then
      simulation_ready=1
      break
    fi
    if ! kill -0 "${simulation_pid}" 2>/dev/null; then
      break
    fi
    sleep 1
  done

  if [[ "${simulation_ready}" -ne 1 ]]; then
    echo "SIMULATION_START_FAILED ${trial}"
    tail -n 30 "${sim_log}" || true
    stop_process_group "${simulation_pid}"
    ACTIVE_SIMULATION_PID=""
    printf '%s,%s,%s,%s,0,,,,,,,,,,,,,,,,\n' \
      "${profile}" "${track_name}" "${repetition}" "${seed}" >> "${ESTIMATOR_SUMMARY}"
    printf '%s,%s,%s,%s,0,,,,,,,,,,,,,,,\n' \
      "${profile}" "${track_name}" "${repetition}" "${seed}" >> "${TRUTH_SUMMARY}"
    return
  fi

  # Allow the EKF one second to establish /odometry/filtered before control.
  sleep 1
  setsid ros2 launch my_robot_controller pid.launch.py \
    config_path:="${config_path}" \
    csv_path:="${track_path}" \
    output_csv_path:="${output_csv}" \
    evaluation_output_csv_path:="${truth_csv}" > "${controller_log}" 2>&1 &
  local controller_pid=$!
  ACTIVE_CONTROLLER_PID="${controller_pid}"

  local complete=0
  for second in $(seq 1 90); do
    if grep -q 'Trajectory complete' "${controller_log}" 2>/dev/null; then
      complete=1
      break
    fi
    if ! kill -0 "${controller_pid}" 2>/dev/null; then
      break
    fi
    if ((second % 15 == 0)); then
      echo "WAIT ${trial} ${second}s"
    fi
    sleep 1
  done

  # The PID includes its exact simulation timestamp in the completion message.
  # Continue simulating for a fixed interval and until Gazebo truth confirms the
  # body is nearly stationary. This replaces the former arbitrary wall-clock
  # sleep, which changed terminal error depending on log-polling phase.
  local completion_stamp=""
  local settled=0
  if [[ "${complete}" -eq 1 ]]; then
    completion_stamp="$(sed -n \
      's/.*Trajectory complete at simulation time \([0-9][0-9.]*\) s.*/\1/p' \
      "${controller_log}" | tail -n 1)"
    if [[ -z "${completion_stamp}" ]]; then
      echo "MISSING_COMPLETION_TIMESTAMP ${trial}"
      complete=0
    fi
  fi

  if [[ "${complete}" -eq 1 ]]; then
    local settling_target
    settling_target="$(awk -v stamp="${completion_stamp}" \
      -v duration="${SETTLING_SIM_TIME}" 'BEGIN {printf "%.9f", stamp + duration}')"
    local attempt
    for attempt in $(seq 1 $((SETTLING_WALL_TIMEOUT * 10))); do
      if [[ -s "${truth_csv}" ]]; then
        local truth_stamp
        local truth_linear_velocity
        local truth_angular_velocity
        IFS=',' read -r truth_stamp truth_linear_velocity truth_angular_velocity < <(
          tail -n 1 "${truth_csv}" | awk -F, '{print $2 "," $33 "," $34}')
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
    echo "SETTLING_TIMEOUT ${trial}"
  fi

  stop_process_group "${controller_pid}"
  ACTIVE_CONTROLLER_PID=""
  stop_process_group "${simulation_pid}"
  ACTIVE_SIMULATION_PID=""

  # Read the expected endpoint directly from the selected track.
  local goal_x
  local goal_y
  IFS=',' read -r goal_x goal_y < <(tail -n 1 "${track_path}" | tr -d '\r')

  if [[ -s "${output_csv}" ]]; then
    # Time-weighted RMSE handles small variations in EKF update intervals.
    # Normalize both commands by their common limits before integration. The
    # result measures dimensionless command activity, not physical motor energy.
    awk -F, \
      -v profile="${profile}" -v track="${track_name}" \
      -v repetition="${repetition}" -v seed="${seed}" -v success="${complete}" \
      -v goal_x="${goal_x}" -v goal_y="${goal_y}" \
      -v linear_limit="1.0" -v angular_limit="1.5" '
      NR == 1 {next}
      {
        count++
        time = $1
        x = $2
        y = $3
        longitudinal = $11
        lateral = $12
        trajectory_heading = $13
        position = $14
        cte = $15
        heading = $16
        v = $20
        w = $21

        abs_cte = cte < 0 ? -cte : cte
        abs_heading = heading < 0 ? -heading : heading
        abs_v = v < 0 ? -v : v
        abs_w = w < 0 ? -w : w
        if (abs_cte > max_cte) max_cte = abs_cte
        if (abs_heading > max_heading) max_heading = abs_heading
        if (abs_v > max_v) max_v = abs_v
        if (abs_w > max_w) max_w = abs_w

        if (count > 1) {
          dt = time - previous_time
          if (dt > 0) {
            cte_squared_integral += cte * cte * dt
            heading_squared_integral += heading * heading * dt
            position_squared_integral += position * position * dt
            longitudinal_squared_integral += longitudinal * longitudinal * dt
            lateral_squared_integral += lateral * lateral * dt
            trajectory_heading_squared_integral += \
              trajectory_heading * trajectory_heading * dt
            command_activity += \
              ((v / linear_limit) ^ 2 + (w / angular_limit) ^ 2) * dt
          }
        }
        previous_time = time
      }
      END {
        if (count > 0 && time > 0) {
          dx = goal_x - x
          dy = goal_y - y
          final_error = sqrt(dx * dx + dy * dy)
          printf "%s,%s,%d,%d,%d,%.6f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.6f,%.6f,%.9f,%.9f,%d\n", \
            profile, track, repetition, seed, success, time, final_error, \
            sqrt(position_squared_integral / time), \
            sqrt(longitudinal_squared_integral / time), \
            sqrt(lateral_squared_integral / time), \
            sqrt(trajectory_heading_squared_integral / time), \
            sqrt(cte_squared_integral / time), max_cte, \
            sqrt(heading_squared_integral / time), max_heading, command_activity, \
            max_v, max_w, x, y, count
        }
      }
    ' "${output_csv}" >> "${ESTIMATOR_SUMMARY}"
  else
    printf '%s,%s,%s,%s,0,,,,,,,,,,,,,,,,\n' \
      "${profile}" "${track_name}" "${repetition}" "${seed}" >> "${ESTIMATOR_SUMMARY}"
  fi

  # Ground truth is used only for scoring. Estimator drift affects the commands
  # and therefore the physical path, while this independent metric prevents the
  # EKF from grading its own estimate as though it were actual robot motion.
  if [[ -s "${truth_csv}" && -s "${output_csv}" ]]; then
    local control_duration
    control_duration="$(tail -n 1 "${output_csv}" | cut -d, -f1)"
    awk -F, \
      -v profile="${profile}" -v track="${track_name}" \
      -v repetition="${repetition}" -v seed="${seed}" \
      -v success="$((complete && settled))" -v control_duration="${control_duration}" \
      -v goal_x="${goal_x}" -v goal_y="${goal_y}" \
      -v goal_tolerance="${TRUTH_GOAL_TOLERANCE}" '
      NR == 1 {next}
      {
        count++
        time = $1
        x = $3
        y = $4
        longitudinal = $11
        lateral = $12
        trajectory_heading = $13
        position = $14
        cte = $18
        heading = $19
        localization = $31
        remaining = $22

        abs_cte = cte < 0 ? -cte : cte
        abs_heading = heading < 0 ? -heading : heading
        if (abs_cte > max_cte) max_cte = abs_cte
        if (abs_heading > max_heading) max_heading = abs_heading
        if (localization != "nan" && localization > max_localization) {
          max_localization = localization
        }

        if (count > 1) {
          dt = time - previous_time
          if (dt > 0) {
            cte_squared_integral += cte * cte * dt
            heading_squared_integral += heading * heading * dt
            position_squared_integral += position * position * dt
            longitudinal_squared_integral += longitudinal * longitudinal * dt
            lateral_squared_integral += lateral * lateral * dt
            trajectory_heading_squared_integral += \
              trajectory_heading * trajectory_heading * dt
            if (localization != "nan") {
              localization_squared_integral += localization * localization * dt
              localization_duration += dt
            }
          }
        }
        previous_time = time
      }
      END {
        if (count > 0 && time > 0) {
          dx = goal_x - x
          dy = goal_y - y
          final_error = sqrt(dx * dx + dy * dy)
          truth_success = (success == 1 && final_error <= goal_tolerance && remaining <= goal_tolerance) ? 1 : 0
          localization_rmse = (localization_duration > 0) ? sqrt(localization_squared_integral / localization_duration) : 0
          printf "%s,%s,%d,%d,%d,%.6f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%d\n", \
            profile, track, repetition, seed, truth_success, control_duration, final_error, \
            sqrt(position_squared_integral / time), \
            sqrt(longitudinal_squared_integral / time), \
            sqrt(lateral_squared_integral / time), \
            sqrt(trajectory_heading_squared_integral / time), \
            sqrt(cte_squared_integral / time), max_cte, \
            sqrt(heading_squared_integral / time), max_heading, \
            localization_rmse, max_localization, x, y, count
        }
      }
    ' "${truth_csv}" >> "${TRUTH_SUMMARY}"
  else
    printf '%s,%s,%s,%s,0,,,,,,,,,,,,,,,\n' \
      "${profile}" "${track_name}" "${repetition}" "${seed}" >> "${TRUTH_SUMMARY}"
  fi

  if [[ "${complete}" -eq 1 ]]; then
    grep 'Trajectory complete' "${controller_log}" | tail -n 1
    echo "DONE ${trial}"
  else
    echo "FAILED ${trial}"
    tail -n 30 "${controller_log}" || true
  fi

  # Let DDS endpoints disappear before starting the next isolated simulation.
  sleep 2
}

read -r -a profiles <<< "${PID_BENCHMARK_PROFILES:-cascade}"
read -r -a tracks <<< "${PID_BENCHMARK_TRACKS:-straight curve corner circle figure_eight}"

for profile in "${profiles[@]}"; do
  for track in "${tracks[@]}"; do
    for repetition in $(seq 1 "${REPETITIONS}"); do
      seed=$((BASE_SEED + repetition - 1))
      case "${track}" in
        straight) run_trial "${profile}" straight "${STRAIGHT_TRACK}" "${repetition}" "${seed}" ;;
        curve) run_trial "${profile}" curve "${CURVE_TRACK}" "${repetition}" "${seed}" ;;
        corner) run_trial "${profile}" corner "${CORNER_TRACK}" "${repetition}" "${seed}" ;;
        circle) run_trial "${profile}" circle "${CIRCLE_TRACK}" "${repetition}" "${seed}" ;;
        figure_eight) run_trial "${profile}" figure_eight "${FIGURE_EIGHT_TRACK}" "${repetition}" "${seed}" ;;
        initial_lateral) run_trial "${profile}" initial_lateral "${INITIAL_LATERAL_TRACK}" "${repetition}" "${seed}" ;;
        initial_heading) run_trial "${profile}" initial_heading "${INITIAL_HEADING_TRACK}" "${repetition}" "${seed}" ;;
        *)
          echo "Unknown flat-world track selector: ${track}" >&2
          exit 2
          ;;
      esac
    done
  done
done

# Incline and rough-terrain worlds are intentionally excluded. Encoder distance
# along a slope is not directly comparable to horizontal 2D path progress, and
# the rough-world boxes are collision steps rather than controlled disturbances.
# Robustness is evaluated through the reproducible command-path fault instead.

echo 'ESTIMATED_STATE_SUMMARY'
cat "${ESTIMATOR_SUMMARY}"
echo 'GROUND_TRUTH_SUMMARY'
cat "${TRUTH_SUMMARY}"

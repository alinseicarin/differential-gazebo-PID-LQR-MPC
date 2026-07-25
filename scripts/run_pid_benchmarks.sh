#!/usr/bin/env bash
# Run named PID profiles on five nominal paths.
#
# Usage from inside the development container:
#   bash scripts/run_pid_benchmarks.sh [result_directory]
#
# Optional space-separated selectors reduce a development run without changing
# the default 3-profile by 5-track formal matrix. For example:
#   PID_BENCHMARK_PROFILES="baseline" \
#   PID_BENCHMARK_TRACKS="circle figure_eight" \
#     bash scripts/run_pid_benchmarks.sh /tmp/pid_closed_paths
#
# The cascaded controller is selected with:
#   PID_BENCHMARK_PROFILES="cascade" \
#     bash scripts/run_pid_benchmarks.sh /tmp/pid_cascade
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

# All three profiles use the first 101 points of the 20 m line, producing a 5 m
# straight trial comparable in scale with the 5 m sinusoidal path.
STRAIGHT_TRACK="${RESULT_DIR}/track_straight_5m.csv"
head -n 101 \
  /home/ws/install/my_robot_controller/share/my_robot_controller/tracks/track_1_straight.csv \
  > "${STRAIGHT_TRACK}"

# The ramp in incline.world emerges from the ground near x=7.8 m. A separate
# 15 m reference ensures that the robot enters and climbs a meaningful length
# of the slope; the ordinary 5 m straight remains the nominal flat benchmark.
INCLINE_STRAIGHT_TRACK="${RESULT_DIR}/track_straight_15m.csv"
head -n 301 \
  /home/ws/install/my_robot_controller/share/my_robot_controller/tracks/track_1_straight.csv \
  > "${INCLINE_STRAIGHT_TRACK}"

CURVE_TRACK="/home/ws/install/my_robot_controller/share/my_robot_controller/tracks/track_2_curve.csv"
CORNER_TRACK="/home/ws/install/my_robot_controller/share/my_robot_controller/tracks/track_3_corner.csv"
CIRCLE_TRACK="/home/ws/install/my_robot_controller/share/my_robot_controller/tracks/track_4_circle.csv"
FIGURE_EIGHT_TRACK="/home/ws/install/my_robot_controller/share/my_robot_controller/tracks/track_5_figure_eight.csv"
INITIAL_LATERAL_TRACK="/home/ws/install/my_robot_controller/share/my_robot_controller/tracks/track_6_initial_lateral_offset.csv"
INITIAL_HEADING_TRACK="/home/ws/install/my_robot_controller/share/my_robot_controller/tracks/track_7_initial_heading_offset.csv"
SUMMARY="${RESULT_DIR}/summary.csv"
ACTIVE_SIMULATION_PID=""
ACTIVE_CONTROLLER_PID=""

printf '%s\n' \
  'profile,track,success,duration,final_error,cte_rmse,cte_max,heading_rmse,heading_max,control_effort,max_v,max_w,final_x,final_y,samples' \
  > "${SUMMARY}"

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
  local world_name="${4:-empty.world}"
  local trial="${profile}_${track_name}"
  local sim_log="${RESULT_DIR}/${trial}_sim.log"
  local controller_log="${RESULT_DIR}/${trial}_controller.log"
  local output_csv="${RESULT_DIR}/${trial}.csv"
  local config_path
  config_path="/home/ws/install/my_robot_controller/share/my_robot_controller/config/pid_${profile}.yaml"

  echo "START ${trial}"

  # setsid gives each launch graph its own process group for reliable cleanup.
  setsid ros2 launch my_robot_description display.launch.py \
    world:="${world_name}" gui:=false > "${sim_log}" 2>&1 &
  local simulation_pid=$!
  ACTIVE_SIMULATION_PID="${simulation_pid}"

  # Wait for the entity rather than sleeping a fixed time. This accommodates
  # different host loads without letting the robot drift unnecessarily.
  local simulation_ready=0
  local second
  for second in $(seq 1 30); do
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
    printf '%s,%s,0,,,,,,,,,,,,\n' "${profile}" "${track_name}" >> "${SUMMARY}"
    return
  fi

  # Allow the EKF one second to establish /odometry/filtered before control.
  sleep 1
  setsid ros2 launch my_robot_controller pid.launch.py \
    config_path:="${config_path}" \
    csv_path:="${track_path}" \
    output_csv_path:="${output_csv}" > "${controller_log}" 2>&1 &
  local controller_pid=$!
  ACTIVE_CONTROLLER_PID="${controller_pid}"

  local complete=0
  for second in $(seq 1 90); do
    if grep -q 'Track complete' "${controller_log}" 2>/dev/null; then
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

  # Give the controller time to flush its terminal CSV row before shutdown.
  sleep 1
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
    # Control effort uses v^2 + 0.2*w^2; the 0.2 weighting keeps angular effort
    # numerically comparable while preserving one definition across all trials.
    awk -F, \
      -v profile="${profile}" -v track="${track_name}" -v success="${complete}" \
      -v goal_x="${goal_x}" -v goal_y="${goal_y}" '
      NR == 1 {next}
      {
        count++
        time = $1
        x = $2
        y = $3
        cte = $8
        heading = $9
        v = $11
        w = $12

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
            control_effort += (v * v + 0.2 * w * w) * dt
          }
        }
        previous_time = time
      }
      END {
        if (count > 0 && time > 0) {
          dx = goal_x - x
          dy = goal_y - y
          final_error = sqrt(dx * dx + dy * dy)
          printf "%s,%s,%d,%.6f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.6f,%.6f,%.9f,%.9f,%d\n", \
            profile, track, success, time, final_error, \
            sqrt(cte_squared_integral / time), max_cte, \
            sqrt(heading_squared_integral / time), max_heading, control_effort, \
            max_v, max_w, x, y, count
        }
      }
    ' "${output_csv}" >> "${SUMMARY}"
  else
    printf '%s,%s,0,,,,,,,,,,,,\n' "${profile}" "${track_name}" >> "${SUMMARY}"
  fi

  if [[ "${complete}" -eq 1 ]]; then
    grep 'Track complete' "${controller_log}" | tail -n 1
    echo "DONE ${trial}"
  else
    echo "FAILED ${trial}"
    tail -n 30 "${controller_log}" || true
  fi

  # Let DDS endpoints disappear before starting the next isolated simulation.
  sleep 2
}

read -r -a profiles <<< "${PID_BENCHMARK_PROFILES:-baseline fast robust}"
read -r -a tracks <<< "${PID_BENCHMARK_TRACKS:-straight curve corner circle figure_eight}"

for profile in "${profiles[@]}"; do
  for track in "${tracks[@]}"; do
    case "${track}" in
      straight) run_trial "${profile}" straight "${STRAIGHT_TRACK}" ;;
      curve) run_trial "${profile}" curve "${CURVE_TRACK}" ;;
      corner) run_trial "${profile}" corner "${CORNER_TRACK}" ;;
      circle) run_trial "${profile}" circle "${CIRCLE_TRACK}" ;;
      figure_eight) run_trial "${profile}" figure_eight "${FIGURE_EIGHT_TRACK}" ;;
      initial_lateral) run_trial "${profile}" initial_lateral "${INITIAL_LATERAL_TRACK}" ;;
      initial_heading) run_trial "${profile}" initial_heading "${INITIAL_HEADING_TRACK}" ;;
      incline_straight) run_trial "${profile}" incline_straight "${INCLINE_STRAIGHT_TRACK}" incline.world ;;
      *)
        echo "Unknown track selector: ${track}" >&2
        exit 2
        ;;
    esac
  done
done

# rough.world is intentionally excluded from this tuning matrix. Its current
# raised boxes behave as collision steps and can throw the robot off the path;
# they must be redesigned as traversable terrain before they provide a fair
# controller-disturbance benchmark. run_trial accepts a fourth world argument
# so an improved world can be added later without changing trial orchestration.

echo 'SUMMARY'
cat "${SUMMARY}"

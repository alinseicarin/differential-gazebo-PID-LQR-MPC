#!/usr/bin/env bash
# Run the predeclared secondary stress campaign on the figure-eight.
#
# Matrix: 3 controllers x 3 scenarios x 10 paired seeds = 90 trials.
# The position relevance threshold is 2.5% of the 0.35 m wheel separation:
# 0.025 * 0.35 m = 0.00875 m. This is fixed before inspecting campaign data.

set -eo pipefail

cd /home/ws

RESULT_DIR="${1:-/home/ws/results/controller_comparison_severe_$(date +%Y%m%d_%H%M%S)}"
REFERENCE_CONFIG="/home/ws/install/my_robot_controller/share/my_robot_controller/config/trajectory_reference_severe.yaml"

COMPARISON_CONTROLLERS="pid lqr mpc" \
COMPARISON_NOMINAL_TRACKS="" \
COMPARISON_ROBUSTNESS_TRACK="figure_eight" \
COMPARISON_ROBUSTNESS_SCENARIOS="nominal angular_pulse_train left_wheel_loss_persistent" \
COMPARISON_REPETITIONS="${SEVERE_COMPARISON_REPETITIONS:-10}" \
COMPARISON_BASE_GAZEBO_SEED="${SEVERE_COMPARISON_BASE_GAZEBO_SEED:-1500}" \
COMPARISON_BASE_NOISE_SEED="${SEVERE_COMPARISON_BASE_NOISE_SEED:-19000}" \
COMPARISON_GUI="${SEVERE_COMPARISON_GUI:-false}" \
COMPARISON_RESUME="${SEVERE_COMPARISON_RESUME:-true}" \
COMPARISON_ROS_DOMAIN_ID="${SEVERE_COMPARISON_ROS_DOMAIN_ID:-92}" \
COMPARISON_STUDY_ROLE="secondary_severe_stress" \
COMPARISON_POSITION_PRACTICAL_THRESHOLD_M="0.00875" \
COMPARISON_HEADING_PRACTICAL_THRESHOLD_RAD="0.02" \
COMPARISON_POSITION_THRESHOLD_BASIS="0.025_x_wheel_separation_0.35_m" \
COMPARISON_POSITION_SENSITIVITY_LOW_M="0.0035" \
COMPARISON_POSITION_SENSITIVITY_HIGH_M="0.0175" \
COMPARISON_ANGULAR_PULSE_START_DELAYS="5.0;11.0;17.0" \
COMPARISON_FIXED_OBSERVATION_DURATION="30.0" \
COMPARISON_REFERENCE_CONFIG_PATH="${REFERENCE_CONFIG}" \
  bash scripts/run_controller_comparison.sh "${RESULT_DIR}"

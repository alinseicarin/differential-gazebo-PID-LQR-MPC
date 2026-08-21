#!/usr/bin/env bash
# Run the independent confirmatory validation of trajectory completion under
# permanent left-wheel effectiveness loss.
#
# Frozen matrix: 3 controllers x 2 conditions x 10 paired seeds = 60 trials.
# The persistent-fault completion endpoint, exact two-sided McNemar test, and
# Holm family were declared before these seeds were observed. Prior campaign
# data are not pooled with this independent validation set.

set -eo pipefail

cd /home/ws

RESULT_DIR="${1:-/home/ws/results/controller_completion_confirmatory_$(date +%Y%m%d_%H%M%S)}"
REFERENCE_CONFIG="/home/ws/install/my_robot_controller/share/my_robot_controller/config/trajectory_reference_severe.yaml"

# This wrapper does not implement trials itself. It freezes a clean independent
# protocol through environment variables, then delegates process isolation,
# pairing, logging, and analysis to the common comparison runner.
COMPARISON_CONTROLLERS="pid lqr mpc" \
COMPARISON_NOMINAL_TRACKS="" \
COMPARISON_ROBUSTNESS_TRACK="figure_eight" \
COMPARISON_ROBUSTNESS_SCENARIOS="nominal left_wheel_loss_persistent" \
COMPARISON_REPETITIONS="${CONFIRMATORY_REPETITIONS:-10}" \
COMPARISON_BASE_GAZEBO_SEED="${CONFIRMATORY_BASE_GAZEBO_SEED:-2500}" \
COMPARISON_BASE_NOISE_SEED="${CONFIRMATORY_BASE_NOISE_SEED:-29000}" \
COMPARISON_GUI="${CONFIRMATORY_GUI:-false}" \
COMPARISON_RESUME="${CONFIRMATORY_RESUME:-true}" \
COMPARISON_ROS_DOMAIN_ID="${CONFIRMATORY_ROS_DOMAIN_ID:-94}" \
COMPARISON_STUDY_ROLE="independent_confirmatory_completion_validation" \
COMPARISON_POSITION_PRACTICAL_THRESHOLD_M="0.00875" \
COMPARISON_HEADING_PRACTICAL_THRESHOLD_RAD="0.02" \
COMPARISON_POSITION_THRESHOLD_BASIS="0.025_x_wheel_separation_0.35_m" \
COMPARISON_POSITION_SENSITIVITY_LOW_M="0.0035" \
COMPARISON_POSITION_SENSITIVITY_HIGH_M="0.0175" \
COMPARISON_FIXED_OBSERVATION_DURATION="30.0" \
COMPARISON_COMPLETION_ANALYSIS_ROLE="predeclared_confirmatory" \
COMPARISON_COMPLETION_CONFIRMATORY_SCENARIOS="left_wheel_loss_persistent" \
COMPARISON_COMPLETION_POSITION_TOLERANCE_M="0.08" \
COMPARISON_COMPLETION_HEADING_TOLERANCE_RAD="0.15" \
COMPARISON_REQUIRE_CLEAN_GIT="true" \
COMPARISON_REFERENCE_CONFIG_PATH="${REFERENCE_CONFIG}" \
  bash scripts/run_controller_comparison.sh "${RESULT_DIR}"

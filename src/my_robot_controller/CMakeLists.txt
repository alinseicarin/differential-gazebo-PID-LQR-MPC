cmake_minimum_required(VERSION 3.8)
project(my_robot_controller)

# ament_cmake provides the standard ROS 2 CMake workflow and package export.
find_package(ament_cmake REQUIRED)

# Enable useful diagnostics without changing generated-code or dependency flags.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Register the linters declared in package.xml when colcon testing is enabled.
# The repository is Apache-2.0 licensed, but per-file copyright headers and the
# project's established C++ formatting are not yet uniform. Keep only those two
# purely stylistic checks disabled; the remaining ament linters still run.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Compile-time ROS interfaces used by pid_node.cpp.
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp_vendor REQUIRED)

# Build controller-independent path geometry as a reusable library. Future PID,
# LQR, and MPC executables must share this exact reference interpretation.
add_library(path_reference_manager src/path_reference_manager.cpp)
target_include_directories(path_reference_manager PUBLIC include)
target_compile_features(path_reference_manager PUBLIC cxx_std_17)

# Convert the geometric path into a deterministic time-indexed virtual robot
# with common speed, yaw-rate, acceleration, and deceleration references.
add_library(trajectory_reference_manager src/trajectory_reference_manager.cpp)
target_include_directories(trajectory_reference_manager PUBLIC include)
target_compile_features(trajectory_reference_manager PUBLIC cxx_std_17)
target_link_libraries(trajectory_reference_manager PUBLIC path_reference_manager)

# The generic PID implementation is shared by the preliminary lookahead loops
# and the thesis cascade, preventing small mathematical differences between
# controller modes.
add_library(pid_controller src/pid_controller.cpp)
target_include_directories(pid_controller PUBLIC include)
target_compile_features(pid_controller PUBLIC cxx_std_17)

# Add both controller corrections to trajectory feedforward using one shared
# safety policy and one set of actuator limits.
add_library(motion_command_policy src/motion_command_policy.cpp)
target_include_directories(motion_command_policy PUBLIC include)
target_compile_features(motion_command_policy PUBLIC cxx_std_17)

# The local state-space model is shared by TVLQR and linear MPC.
# Exact zero-order-hold discretization uses Eigen's matrix exponential.
add_library(linearized_error_model src/linearized_error_model.cpp)
target_include_directories(linearized_error_model PUBLIC include)
target_compile_features(linearized_error_model PUBLIC cxx_std_17)
target_link_libraries(linearized_error_model PUBLIC Eigen3::Eigen)

# Finite-horizon time-varying LQR is kept independent of ROS so the Riccati
# recursion, signs, and gain schedule can be verified in unit tests.
add_library(time_varying_lqr_controller src/time_varying_lqr_controller.cpp)
target_include_directories(time_varying_lqr_controller PUBLIC include)
target_compile_features(time_varying_lqr_controller PUBLIC cxx_std_17)
target_link_libraries(
  time_varying_lqr_controller PUBLIC linearized_error_model Eigen3::Eigen)

# The constrained linear MPC is also independent of ROS. It assembles the
# finite-horizon LTV problem and delegates the convex QP solve to OSQP.
add_library(linear_mpc_controller src/linear_mpc_controller.cpp)
target_include_directories(linear_mpc_controller PUBLIC include)
target_compile_features(linear_mpc_controller PUBLIC cxx_std_17)
target_link_libraries(
  linear_mpc_controller PUBLIC linearized_error_model Eigen3::Eigen osqp::osqp)

# Keep the cascade free of ROS interfaces so its signs, immediate heading
# response, and feedback calculation can be tested without launching Gazebo.
add_library(cascaded_pid_controller src/cascaded_pid_controller.cpp)
target_include_directories(cascaded_pid_controller PUBLIC include)
target_compile_features(cascaded_pid_controller PUBLIC cxx_std_17)
target_link_libraries(
  cascaded_pid_controller PUBLIC pid_controller path_reference_manager)

# The downstream command fault is controller-independent. It can therefore be
# reused unchanged when PID is replaced by LQR or MPC.
add_library(command_disturbance src/command_disturbance.cpp)
target_include_directories(command_disturbance PUBLIC include)
target_compile_features(command_disturbance PUBLIC cxx_std_17)

# Reproducible measurement-path corruption is isolated from ROS messages so
# its schedule, noise seed, and angular wrapping can be unit tested directly.
add_library(odometry_disturbance src/odometry_disturbance.cpp)
target_include_directories(odometry_disturbance PUBLIC include)
target_compile_features(odometry_disturbance PUBLIC cxx_std_17)
target_link_libraries(odometry_disturbance PUBLIC path_reference_manager)

# Convert wheel-joint rotation and Gazebo body twist into explicit slip metrics
# without coupling the calculation to ROS subscriptions or CSV plumbing.
add_library(wheel_slip_metrics src/wheel_slip_metrics.cpp)
target_include_directories(wheel_slip_metrics PUBLIC include)
target_compile_features(wheel_slip_metrics PUBLIC cxx_std_17)

# Build the selectable PID executable against the reusable control components.
add_executable(pid_node src/pid_node.cpp)
target_include_directories(pid_node PUBLIC include)
target_compile_features(pid_node PUBLIC cxx_std_17)
target_link_libraries(
  pid_node path_reference_manager pid_controller cascaded_pid_controller
  motion_command_policy trajectory_reference_manager)
ament_target_dependencies(pid_node rclcpp geometry_msgs nav_msgs std_msgs)

# TVLQR uses the same timed reference and final command policy as PID while
# keeping its ROS transport and experiment log in a separate executable.
add_executable(lqr_node src/lqr_node.cpp)
target_include_directories(lqr_node PUBLIC include)
target_compile_features(lqr_node PUBLIC cxx_std_17)
target_link_libraries(
  lqr_node trajectory_reference_manager motion_command_policy
  time_varying_lqr_controller linearized_error_model Eigen3::Eigen)
ament_target_dependencies(lqr_node rclcpp geometry_msgs nav_msgs std_msgs)

# LTV-MPC shares the LQR model and timed reference but solves a constrained QP
# online at every odometry update.
add_executable(mpc_node src/mpc_node.cpp)
target_include_directories(mpc_node PUBLIC include)
target_compile_features(mpc_node PUBLIC cxx_std_17)
target_link_libraries(
  mpc_node trajectory_reference_manager motion_command_policy
  linear_mpc_controller linearized_error_model Eigen3::Eigen osqp::osqp)
ament_target_dependencies(mpc_node rclcpp geometry_msgs nav_msgs std_msgs)

# ROS adapter that converts /cmd_vel_nominal into the actual /cmd_vel while
# applying and logging one simulation-time command fault.
add_executable(
  command_disturbance_injector src/command_disturbance_injector_node.cpp)
target_include_directories(command_disturbance_injector PUBLIC include)
target_compile_features(command_disturbance_injector PUBLIC cxx_std_17)
target_link_libraries(command_disturbance_injector command_disturbance)
ament_target_dependencies(command_disturbance_injector rclcpp geometry_msgs std_msgs)

# Adapter placed after the EKF and before a controller. Ground-truth evaluation
# remains connected to the clean estimate and therefore outside this fault path.
add_executable(
  odometry_disturbance_injector src/odometry_disturbance_injector_node.cpp)
target_include_directories(odometry_disturbance_injector PUBLIC include)
target_compile_features(odometry_disturbance_injector PUBLIC cxx_std_17)
target_link_libraries(odometry_disturbance_injector odometry_disturbance)
ament_target_dependencies(odometry_disturbance_injector rclcpp nav_msgs std_msgs)

# Independent evaluator: controllers consume only EKF state, while this node
# compares the estimate and physical trajectory against Gazebo world truth.
add_executable(trajectory_evaluator_node src/trajectory_evaluator_node.cpp)
target_include_directories(trajectory_evaluator_node PUBLIC include)
target_compile_features(trajectory_evaluator_node PUBLIC cxx_std_17)
target_link_libraries(
  trajectory_evaluator_node trajectory_reference_manager wheel_slip_metrics)
ament_target_dependencies(
  trajectory_evaluator_node rclcpp nav_msgs sensor_msgs std_msgs)

if(BUILD_TESTING)
  # Geometry tests run without ROS or Gazebo and protect the common definitions
  # used in every later controller comparison.
  ament_add_gtest(test_path_reference_manager test/test_path_reference_manager.cpp)
  target_link_libraries(test_path_reference_manager path_reference_manager)
  target_include_directories(test_path_reference_manager PUBLIC include)
  target_compile_features(test_path_reference_manager PUBLIC cxx_std_17)

  # Freeze time parameterization, body-frame pose errors, and independence of
  # the virtual reference from actual robot progress.
  ament_add_gtest(
    test_trajectory_reference_manager test/test_trajectory_reference_manager.cpp)
  target_link_libraries(
    test_trajectory_reference_manager trajectory_reference_manager)
  target_include_directories(test_trajectory_reference_manager PUBLIC include)
  target_compile_features(test_trajectory_reference_manager PUBLIC cxx_std_17)

  # These tests directly verify the cascaded control law, including the case
  # where a robot is rotated while still geometrically on the path.
  ament_add_gtest(test_cascaded_pid_controller test/test_cascaded_pid_controller.cpp)
  target_link_libraries(test_cascaded_pid_controller cascaded_pid_controller)
  target_include_directories(test_cascaded_pid_controller PUBLIC include)
  target_compile_features(test_cascaded_pid_controller PUBLIC cxx_std_17)

  # Freeze the controller-independent speed, feedforward, safety, and command
  # limits that must be reused for the PID/LQR/MPC comparison.
  ament_add_gtest(test_motion_command_policy test/test_motion_command_policy.cpp)
  target_link_libraries(test_motion_command_policy motion_command_policy)
  target_include_directories(test_motion_command_policy PUBLIC include)
  target_compile_features(test_motion_command_policy PUBLIC cxx_std_17)

  # Protect the sign convention, exact ZOH discretization, and dependence on
  # the two time-varying reference velocities.
  ament_add_gtest(test_linearized_error_model test/test_linearized_error_model.cpp)
  target_link_libraries(test_linearized_error_model linearized_error_model)
  target_include_directories(test_linearized_error_model PUBLIC include)
  target_compile_features(test_linearized_error_model PUBLIC cxx_std_17)

  # Verify the finite-horizon Riccati schedule before adding ROS or Gazebo.
  ament_add_gtest(
    test_time_varying_lqr_controller test/test_time_varying_lqr_controller.cpp)
  target_link_libraries(
    test_time_varying_lqr_controller time_varying_lqr_controller)
  target_include_directories(test_time_varying_lqr_controller PUBLIC include)
  target_compile_features(test_time_varying_lqr_controller PUBLIC cxx_std_17)

  # Verify zero-error equilibrium, feedback signs, and hard absolute-command
  # bounds in the ROS-independent MPC optimization problem.
  ament_add_gtest(test_linear_mpc_controller test/test_linear_mpc_controller.cpp)
  target_link_libraries(test_linear_mpc_controller linear_mpc_controller)
  target_include_directories(test_linear_mpc_controller PUBLIC include)
  target_compile_features(test_linear_mpc_controller PUBLIC cxx_std_17)

  # Boundary, bias, and saturation tests freeze the exact fault supplied to all
  # controller families in the final comparison.
  ament_add_gtest(test_command_disturbance test/test_command_disturbance.cpp)
  target_link_libraries(test_command_disturbance command_disturbance)
  target_include_directories(test_command_disturbance PUBLIC include)
  target_compile_features(test_command_disturbance PUBLIC cxx_std_17)

  # Validate schedule boundaries, deterministic random seeds, bias signs, and
  # rejection of invalid measurement-noise configurations.
  ament_add_gtest(test_odometry_disturbance test/test_odometry_disturbance.cpp)
  target_link_libraries(test_odometry_disturbance odometry_disturbance)
  target_include_directories(test_odometry_disturbance PUBLIC include)
  target_compile_features(test_odometry_disturbance PUBLIC cxx_std_17)

  # Check that ordinary differential-drive turning is not mistaken for slip
  # and that longitudinal and lateral discrepancies retain their definitions.
  ament_add_gtest(test_wheel_slip_metrics test/test_wheel_slip_metrics.cpp)
  target_link_libraries(test_wheel_slip_metrics wheel_slip_metrics)
  target_include_directories(test_wheel_slip_metrics PUBLIC include)
  target_compile_features(test_wheel_slip_metrics PUBLIC cxx_std_17)

endif()

# ROS 2 discovers executables installed under lib/<package-name>.
install(TARGETS
  pid_node
  lqr_node
  mpc_node
  command_disturbance_injector
  odometry_disturbance_injector
  trajectory_evaluator_node
  path_reference_manager
  trajectory_reference_manager
  pid_controller
  motion_command_policy
  linearized_error_model
  time_varying_lqr_controller
  linear_mpc_controller
  cascaded_pid_controller
  command_disturbance
  odometry_disturbance
  wheel_slip_metrics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install the common geometry API for the later LQR and MPC executables.
install(DIRECTORY include/ DESTINATION include)

# Install runtime configuration and launch descriptions into the package share.
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# Copy the workspace-level benchmark tracks into the installed package. The PID
# launch file can then locate them through the ament index from any directory.
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../../track_1_straight.csv
  ${CMAKE_CURRENT_SOURCE_DIR}/../../track_2_curve.csv
  ${CMAKE_CURRENT_SOURCE_DIR}/../../track_3_corner.csv
  ${CMAKE_CURRENT_SOURCE_DIR}/../../track_4_circle.csv
  ${CMAKE_CURRENT_SOURCE_DIR}/../../track_5_figure_eight.csv
  ${CMAKE_CURRENT_SOURCE_DIR}/../../track_6_initial_lateral_offset.csv
  ${CMAKE_CURRENT_SOURCE_DIR}/../../track_7_initial_heading_offset.csv
  DESTINATION share/${PROJECT_NAME}/tracks
)

# Generate and export the package metadata consumed by colcon and ros2 tools.
ament_export_include_directories(include)
ament_export_libraries(
  path_reference_manager
  trajectory_reference_manager
  pid_controller
  motion_command_policy
  linearized_error_model
  time_varying_lqr_controller
  linear_mpc_controller
  cascaded_pid_controller
  command_disturbance
  odometry_disturbance)
ament_export_dependencies(eigen3_cmake_module Eigen3 osqp_vendor)
ament_package()

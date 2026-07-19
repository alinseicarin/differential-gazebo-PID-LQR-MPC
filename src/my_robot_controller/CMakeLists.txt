cmake_minimum_required(VERSION 3.8)
project(my_robot_controller)

# ament_cmake provides the standard ROS 2 CMake workflow and package export.
find_package(ament_cmake REQUIRED)

# Enable useful diagnostics without changing generated-code or dependency flags.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Register the linters declared in package.xml when colcon testing is enabled.
# Copyright and cpplint remain disabled until project-wide license headers are
# selected and added consistently.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Compile-time ROS interfaces used by pid_node.cpp.
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Build the controller as a ROS 2 executable. C++17 is required for std::clamp.
add_executable(pid_node src/pid_node.cpp)
target_include_directories(pid_node PUBLIC include)
target_compile_features(pid_node PUBLIC cxx_std_17)
ament_target_dependencies(pid_node rclcpp geometry_msgs nav_msgs)

# ROS 2 discovers executables installed under lib/<package-name>.
install(TARGETS
  pid_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install runtime configuration and launch descriptions into the package share.
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Copy the workspace-level benchmark tracks into the installed package. The PID
# launch file can then locate them through the ament index from any directory.
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../../track_1_straight.csv
  ${CMAKE_CURRENT_SOURCE_DIR}/../../track_2_curve.csv
  ${CMAKE_CURRENT_SOURCE_DIR}/../../track_3_corner.csv
  ${CMAKE_CURRENT_SOURCE_DIR}/../../track_4_circle.csv
  ${CMAKE_CURRENT_SOURCE_DIR}/../../track_5_figure_eight.csv
  DESTINATION share/${PROJECT_NAME}/tracks
)

# Generate and export the package metadata consumed by colcon and ros2 tools.
ament_package()

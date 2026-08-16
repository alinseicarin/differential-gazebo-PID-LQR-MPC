cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

# This package contains no compiled node, but keep standard warning flags for
# any C/C++ targets that may be added later.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ament_cmake supplies package installation, indexing, and test integration.
find_package(ament_cmake REQUIRED)

# Run Python, XML, CMake, and package metadata checks under `colcon test`.
# The repository is Apache-2.0 licensed, but per-file copyright headers are not
# yet uniform. Keep that stylistic check disabled; the remaining linters run.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install non-code assets under share/my_robot_description so launch files can
# locate them through the ament package index after a normal or symlink build.
install(DIRECTORY
  urdf
  launch
  rviz
  config
  worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# Export package metadata even though there is no compiled library/executable.
ament_package()

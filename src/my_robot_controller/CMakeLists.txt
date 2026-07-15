cmake_minimum_required(VERSION 3.8)
project(my_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# 1. Tell CMake to compile your C++ file into an executable named 'pid_node'
add_executable(pid_node src/pid_node.cpp)

# Add this line so the compiler can find your .hpp file!
target_include_directories(pid_node PUBLIC include)

# 2. Link the ROS 2 libraries to your executable
ament_target_dependencies(pid_node rclcpp geometry_msgs nav_msgs)

# 3. Tell colcon where to install the finished node so ros2 run can find it
install(TARGETS
  pid_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

"""Launch the PID path follower with reproducible experiment parameters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Declare experiment arguments and construct the PID ROS node action."""
    # FindPackageShare resolves the installed package through the ament index;
    # this avoids hard-coded /home/ws paths and works after relocating a build.
    package_share = FindPackageShare('my_robot_controller')

    # LaunchConfiguration objects are resolved only when the launch description
    # executes, allowing terminal arguments to override every default below.
    csv_path = LaunchConfiguration('csv_path')
    output_csv_path = LaunchConfiguration('output_csv_path')
    config_path = LaunchConfiguration('config_path')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # The default track is installed by CMake alongside this launch file.
        DeclareLaunchArgument(
            'csv_path',
            default_value=PathJoinSubstitution([
                package_share, 'tracks', 'track_1_straight.csv'
            ]),
            description='CSV file containing headerless x,y waypoint rows',
        ),
        # A relative output path is interpreted from the directory in which the
        # user invokes ros2 launch. Pass an absolute path for scripted trials.
        DeclareLaunchArgument(
            'output_csv_path',
            default_value='robot_actual_trajectory.csv',
            description='Destination for timestamped controller experiment data',
        ),
        # Alternate YAML files make named tuning sets possible without editing
        # the controller's source or its baseline configuration.
        DeclareLaunchArgument(
            'config_path',
            default_value=PathJoinSubstitution([
                package_share, 'config', 'pid_baseline.yaml'
            ]),
            description='PID profile YAML (baseline, fast, robust, cascade, or custom)',
        ),
        # Gazebo experiments should use /clock. Setting this false also permits
        # focused tests driven by odometry with ordinary wall-clock timestamps.
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Gazebo simulation clock',
        ),
        # YAML supplies the baseline settings; the following dictionary is
        # applied later and therefore gives launch arguments override priority.
        Node(
            package='my_robot_controller',
            executable='pid_node',
            name='pid_node',
            output='screen',
            parameters=[
                config_path,
                {
                    'csv_path': csv_path,
                    'output_csv_path': output_csv_path,
                    # Explicit type conversion avoids treating "true" as text.
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                },
            ],
        ),
    ])

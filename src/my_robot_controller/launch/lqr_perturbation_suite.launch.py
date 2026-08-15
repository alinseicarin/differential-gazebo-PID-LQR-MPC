"""Launch TVLQR through the same command and odometry fault paths as PID."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _float(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _bool(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def _int(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def generate_launch_description():
    """Construct clean evaluation and perturbed TVLQR feedback/command paths."""
    package_share = FindPackageShare('my_robot_controller')
    arguments = [
        DeclareLaunchArgument(
            'csv_path',
            default_value=PathJoinSubstitution([
                package_share, 'tracks', 'track_5_figure_eight.csv'
            ]),
        ),
        DeclareLaunchArgument(
            'config_path',
            default_value=PathJoinSubstitution([
                package_share, 'config', 'lqr.yaml'
            ]),
        ),
        DeclareLaunchArgument(
            'reference_config_path',
            default_value=PathJoinSubstitution([
                package_share, 'config', 'trajectory_reference.yaml'
            ]),
        ),
        DeclareLaunchArgument('controller_output_csv_path'),
        DeclareLaunchArgument('applied_command_csv_path'),
        DeclareLaunchArgument('disturbed_odometry_csv_path'),
        DeclareLaunchArgument('evaluation_output_csv_path'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('command_fault_enabled', default_value='false'),
        DeclareLaunchArgument('feedback_fault_enabled', default_value='false'),
        DeclareLaunchArgument('fault_start_delay', default_value='5.0'),
        DeclareLaunchArgument('fault_duration', default_value='1.0'),
        DeclareLaunchArgument('linear_velocity_bias', default_value='0.0'),
        DeclareLaunchArgument('angular_velocity_bias', default_value='0.0'),
        DeclareLaunchArgument('left_wheel_effectiveness', default_value='1.0'),
        DeclareLaunchArgument('right_wheel_effectiveness', default_value='1.0'),
        DeclareLaunchArgument('wheel_separation', default_value='0.35'),
        DeclareLaunchArgument('command_delay', default_value='0.0'),
        DeclareLaunchArgument('odometry_x_bias', default_value='0.0'),
        DeclareLaunchArgument('odometry_y_bias', default_value='0.0'),
        DeclareLaunchArgument('odometry_yaw_bias', default_value='0.0'),
        DeclareLaunchArgument('position_noise_stddev', default_value='0.0'),
        DeclareLaunchArgument('yaw_noise_stddev', default_value='0.0'),
        DeclareLaunchArgument('noise_seed', default_value='2026'),
    ]

    controller = Node(
        package='my_robot_controller',
        executable='lqr_node',
        name='lqr_node',
        output='screen',
        remappings=[
            ('cmd_vel', 'cmd_vel_nominal'),
            ('odometry/filtered', 'odometry/filtered_disturbed'),
        ],
        parameters=[
            LaunchConfiguration('config_path'),
            LaunchConfiguration('reference_config_path'),
            {
                'csv_path': LaunchConfiguration('csv_path'),
                'output_csv_path': LaunchConfiguration(
                    'controller_output_csv_path'
                ),
                'use_sim_time': _bool('use_sim_time'),
            },
        ],
    )

    command_injector = Node(
        package='my_robot_controller',
        executable='command_disturbance_injector',
        name='command_disturbance_injector',
        output='screen',
        parameters=[{
            'use_sim_time': _bool('use_sim_time'),
            'fault_enabled': _bool('command_fault_enabled'),
            'fault_start_delay': _float('fault_start_delay'),
            'fault_duration': _float('fault_duration'),
            'linear_velocity_bias': _float('linear_velocity_bias'),
            'angular_velocity_bias': _float('angular_velocity_bias'),
            'left_wheel_effectiveness': _float('left_wheel_effectiveness'),
            'right_wheel_effectiveness': _float('right_wheel_effectiveness'),
            'wheel_separation': _float('wheel_separation'),
            'command_delay': _float('command_delay'),
            'maximum_abs_linear_velocity': 1.0,
            'maximum_abs_angular_velocity': 1.5,
            'input_timeout': 2.0,
            'output_csv_path': LaunchConfiguration(
                'applied_command_csv_path'
            ),
        }],
    )

    odometry_injector = Node(
        package='my_robot_controller',
        executable='odometry_disturbance_injector',
        name='odometry_disturbance_injector',
        output='screen',
        parameters=[{
            'use_sim_time': _bool('use_sim_time'),
            'fault_enabled': _bool('feedback_fault_enabled'),
            'fault_start_delay': _float('fault_start_delay'),
            'fault_duration': _float('fault_duration'),
            'x_bias': _float('odometry_x_bias'),
            'y_bias': _float('odometry_y_bias'),
            'yaw_bias': _float('odometry_yaw_bias'),
            'position_noise_standard_deviation': _float(
                'position_noise_stddev'
            ),
            'yaw_noise_standard_deviation': _float('yaw_noise_stddev'),
            'random_seed': _int('noise_seed'),
            'output_csv_path': LaunchConfiguration(
                'disturbed_odometry_csv_path'
            ),
        }],
    )

    evaluator = Node(
        package='my_robot_controller',
        executable='trajectory_evaluator_node',
        name='trajectory_evaluator_node',
        output='screen',
        parameters=[
            LaunchConfiguration('reference_config_path'),
            {
                'csv_path': LaunchConfiguration('csv_path'),
                'output_csv_path': LaunchConfiguration(
                    'evaluation_output_csv_path'
                ),
                'use_sim_time': _bool('use_sim_time'),
            },
        ],
    )

    return LaunchDescription(
        arguments + [controller, command_injector, odometry_injector, evaluator]
    )

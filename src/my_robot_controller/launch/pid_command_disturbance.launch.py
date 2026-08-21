"""Launch PID through a reproducible downstream command-fault injector."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Construct the controller -> fault injector -> Gazebo command path."""
    package_share = FindPackageShare('my_robot_controller')

    csv_path = LaunchConfiguration('csv_path')
    controller_output_csv_path = LaunchConfiguration(
        'controller_output_csv_path'
    )
    applied_command_csv_path = LaunchConfiguration(
        'applied_command_csv_path'
    )
    evaluation_output_csv_path = LaunchConfiguration(
        'evaluation_output_csv_path'
    )
    config_path = LaunchConfiguration('config_path')
    reference_config_path = LaunchConfiguration('reference_config_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    fault_start_delay = LaunchConfiguration('fault_start_delay')
    fault_duration = LaunchConfiguration('fault_duration')
    linear_velocity_bias = LaunchConfiguration('linear_velocity_bias')
    angular_velocity_bias = LaunchConfiguration('angular_velocity_bias')

    return LaunchDescription([
        DeclareLaunchArgument(
            'csv_path',
            default_value=PathJoinSubstitution([
                package_share, 'tracks', 'track_1_straight.csv'
            ]),
            description='Headerless x,y path used by the controller',
        ),
        DeclareLaunchArgument(
            'controller_output_csv_path',
            default_value='robot_actual_trajectory.csv',
            description='Controller state/reference/nominal-command CSV',
        ),
        DeclareLaunchArgument(
            'applied_command_csv_path',
            default_value='command_disturbance_actual_commands.csv',
            description='Nominal versus applied actuator-command CSV',
        ),
        DeclareLaunchArgument(
            'evaluation_output_csv_path',
            default_value='robot_ground_truth_trajectory.csv',
            description='Ground-truth tracking and EKF localization-error CSV',
        ),
        DeclareLaunchArgument(
            'config_path',
            default_value=PathJoinSubstitution([
                package_share, 'config', 'pid_cascade.yaml'
            ]),
            description='Controller configuration, normally pid_cascade.yaml',
        ),
        DeclareLaunchArgument(
            'reference_config_path',
            default_value=PathJoinSubstitution([
                package_share, 'config', 'trajectory_reference.yaml'
            ]),
            description='Common timed-reference configuration',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation time in both nodes',
        ),
        DeclareLaunchArgument(
            'fault_start_delay',
            default_value='5.0',
            description='Seconds after the synchronized experiment start before fault',
        ),
        DeclareLaunchArgument(
            'fault_duration',
            default_value='1.0',
            description='Command-fault duration in simulation seconds',
        ),
        DeclareLaunchArgument(
            'linear_velocity_bias',
            default_value='0.0',
            description='Additive forward-velocity fault in m/s',
        ),
        DeclareLaunchArgument(
            'angular_velocity_bias',
            default_value='0.6',
            description='Additive yaw-rate fault in rad/s',
        ),
        # Remap only this controller output. The ordinary pid.launch.py remains
        # a direct /cmd_vel path for all nominal benchmark runs.
        Node(
            package='my_robot_controller',
            executable='pid_node',
            name='pid_node',
            output='screen',
            remappings=[('cmd_vel', 'cmd_vel_nominal')],
            parameters=[
                config_path,
                reference_config_path,
                {
                    'csv_path': csv_path,
                    'output_csv_path': controller_output_csv_path,
                    'use_sim_time': ParameterValue(
                        use_sim_time, value_type=bool
                    ),
                },
            ],
        ),
        Node(
            package='my_robot_controller',
            executable='command_disturbance_injector',
            name='command_disturbance_injector',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool
                ),
                'fault_start_delay': ParameterValue(
                    fault_start_delay, value_type=float
                ),
                'fault_duration': ParameterValue(
                    fault_duration, value_type=float
                ),
                'linear_velocity_bias': ParameterValue(
                    linear_velocity_bias, value_type=float
                ),
                'angular_velocity_bias': ParameterValue(
                    angular_velocity_bias, value_type=float
                ),
                'maximum_abs_linear_velocity': 1.0,
                'maximum_abs_angular_velocity': 1.5,
                'input_timeout': 2.0,
                'output_csv_path': applied_command_csv_path,
            }],
        ),
        # Evaluation-only access to Gazebo truth remains isolated from both the
        # controller and command-fault path.
        Node(
            package='my_robot_controller',
            executable='trajectory_evaluator_node',
            name='trajectory_evaluator_node',
            output='screen',
            parameters=[
                reference_config_path,
                {
                    'csv_path': csv_path,
                    'output_csv_path': evaluation_output_csv_path,
                    'use_sim_time': ParameterValue(
                        use_sim_time, value_type=bool
                    ),
                },
            ],
        ),
    ])

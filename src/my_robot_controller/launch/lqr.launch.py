"""Launch timed TVLQR tracking with the independent ground-truth evaluator."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Declare common experiment paths and construct the LQR control graph."""
    package_share = FindPackageShare('my_robot_controller')
    csv_path = LaunchConfiguration('csv_path')
    output_csv_path = LaunchConfiguration('output_csv_path')
    evaluation_output_csv_path = LaunchConfiguration(
        'evaluation_output_csv_path'
    )
    config_path = LaunchConfiguration('config_path')
    reference_config_path = LaunchConfiguration('reference_config_path')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'csv_path',
            default_value=PathJoinSubstitution([
                package_share, 'tracks', 'track_1_straight.csv'
            ]),
            description='CSV file containing headerless x,y waypoint rows',
        ),
        DeclareLaunchArgument(
            'output_csv_path',
            default_value='robot_lqr_trajectory.csv',
            description='Destination for TVLQR state, gain, and command data',
        ),
        DeclareLaunchArgument(
            'evaluation_output_csv_path',
            default_value='robot_lqr_ground_truth_trajectory.csv',
            description='Ground-truth tracking and localization evaluation CSV',
        ),
        DeclareLaunchArgument(
            'config_path',
            default_value=PathJoinSubstitution([
                package_share, 'config', 'lqr.yaml'
            ]),
            description='TVLQR weights and common experiment parameters',
        ),
        DeclareLaunchArgument(
            'reference_config_path',
            default_value=PathJoinSubstitution([
                package_share, 'config', 'trajectory_reference.yaml'
            ]),
            description='Common timed-reference configuration',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='my_robot_controller',
            executable='lqr_node',
            name='lqr_node',
            output='screen',
            parameters=[
                config_path,
                reference_config_path,
                {
                    'csv_path': csv_path,
                    'output_csv_path': output_csv_path,
                    'use_sim_time': ParameterValue(
                        use_sim_time, value_type=bool
                    ),
                },
            ],
        ),
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

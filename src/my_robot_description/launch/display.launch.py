"""Launch the robot model, Gazebo simulation, EKF, and RViz visualization."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Construct the complete simulation and state-estimation launch graph."""
    # Resolve installed assets through the ament index. This works for both
    # normal and --symlink-install builds without hard-coded workspace paths.
    pkg_path = get_package_share_directory('my_robot_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'my_robot.urdf')

    # robot_state_publisher and Gazebo's entity spawner both consume the same
    # URDF text from the transient-local /robot_description topic.
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Select one of the installed SDF environments from the command line, e.g.
    # `world:=rough.world`. The flat empty world is the nominal baseline.
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.world',  # Your flat testing track
        description='The Gazebo world file to load (empty.world, incline.world, rough.world)'
    )

    # Automated experiments normally disable GUI rendering so Gazebo can keep
    # up with simulation time and produce repeatable wall-clock behavior.
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo and RViz GUIs; false runs physics headlessly'
    )

    # A fixed default seed makes simulated sensor noise reproducible. Formal
    # repeated trials can override it to quantify sensitivity to noise.
    seed_arg = DeclareLaunchArgument(
        'seed',
        default_value='42',
        description='Gazebo random seed used by physics and sensor noise'
    )

    # LaunchConfiguration is resolved at launch time after arguments are parsed.
    world_name = LaunchConfiguration('world')
    seed = LaunchConfiguration('seed')

    # A substitution list concatenates the directory and selected world name.
    world_path = [os.path.join(pkg_path, 'worlds', ''), world_name]
    gazebo_config_path = os.path.join(pkg_path, 'config', 'gazebo.yaml')

    # Publish fixed URDF joints and wheel transforms derived from /joint_states.
    # Simulation time keeps TF timestamps aligned with Gazebo and the EKF.
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': True},
        ]
    )

    # RViz is visualization only; it does not feed commands or simulation state.
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'display.rviz')

    # Load a repeatable camera/display layout with odom as the fixed frame.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # Interactive process: `gazebo` starts both gzserver and gzclient.
    gazebo_gui = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             '--seed', seed,
             '--ros-args', '--params-file', gazebo_config_path, '--'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # Headless process: physics and ROS plugins run without rendering a client.
    gazebo_headless = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_path,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             '--seed', seed,
             '--ros-args', '--params-file', gazebo_config_path, '--'],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    # Insert one robot model from /robot_description. spawn_entity.py waits for
    # Gazebo's service, so explicit timer-based launch sequencing is unnecessary.
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot', '-topic', 'robot_description'],
        output='screen'
    )

    # robot_localization fuses wheel odometry and IMU measurements at 30 Hz.
    ekf_config_path = os.path.join(pkg_path, 'config', 'ekf.yaml')

    # Its output /odometry/filtered drives the PID controller, and it is the sole
    # publisher of the dynamic odom -> base_footprint transform.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}]
    )

    # Launch actions start concurrently. Components with service/topic
    # dependencies wait internally for their required Gazebo/ROS interfaces.
    return LaunchDescription([
        world_arg,
        gui_arg,
        seed_arg,
        rsp_node,
        # RViz is unnecessary in headless benchmark mode.
        rviz_node,
        gazebo_gui,
        gazebo_headless,
        spawn_robot_node,
        ekf_node
    ])

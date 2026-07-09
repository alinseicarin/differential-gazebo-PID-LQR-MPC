import os
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Find your package and the URDF file
    pkg_path = get_package_share_directory('my_robot_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'my_robot.urdf')

    # Read the URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Node 1: robot_state_publisher (Broadcasts the URDF math to the system)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Node 2: joint_state_publisher_gui (Gives you a popup window with sliders to spin the wheels!)
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # Find the RViz config file
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'display.rviz')

    # Node 3: RViz2 (The visualizer)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] # <--- This tells it to load your save file!
    )

    # Node 4: Boot up the Gazebo Physics World
    gazebo_server= IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')])
    )

    # Node 5: Drop your robot URDF into the world
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot', '-topic', 'robot_description'],
        output='screen'
    )

    # Find the config file
    ekf_config_path = os.path.join(pkg_path, 'config', 'ekf.yaml')

    # Node 6: The Extended Kalman Filter
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        # Add the dictionary {'use_sim_time': True} to this list!
        parameters=[ekf_config_path, {'use_sim_time': True}]
    )

    return LaunchDescription([
        rsp_node,
        jsp_gui_node,
        rviz_node,
        gazebo_server,
        spawn_robot_node,
        ekf_node
    ])
# Launch file for Mr. PeterBot — Gazebo + RViz + ros2_control

# Libraries and dependencies
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessExit
import xacro

# Define the launch
def generate_launch_description():

    # Get package paths
    pkg_description = get_package_share_directory('mr_peterbot_description')
    pkg_bringup = get_package_share_directory('mr_peterbot_bringup')

    urdf_file = os.path.join(pkg_description, 'urdf', 'mr_peterbot.urdf.xacro')
    world_file = os.path.join(pkg_description, 'worlds', 'empty.sdf')

    # Load the Xacro file
    robot_description = xacro.process_file(urdf_file).toxml()

    # Get controllers configuration path
    controllers_yaml = os.path.join(pkg_bringup, 'config', 'controllers.yaml')

    return LaunchDescription([

        # Launch Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen'
        ),

        # Robot State Publisher NODE
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description,
            'use_sim_time': True}]
        ),

        # Spawn MrPeterBot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_mr_peterbot',
            arguments=[
                '-name', 'mr_peterbot',
                '-topic', '/robot_description',
                '-z', '0.05'
            ],
            output='screen'
        ),

        # Controller Manager
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster',
                       '--param-file', controllers_yaml],
            output='screen'
        ),

        # Diff Drive Controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller',
                       '--param-file', controllers_yaml],
            output='screen'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        ),

        # Clock bridge — Gazebo → ROS2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        ),
    ])
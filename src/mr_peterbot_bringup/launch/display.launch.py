import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            RegisterEventHandler, ExecuteProcess)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    # ===== PATHS =====
    description_pkg = get_package_share_directory('mr_peterbot_description')
    bringup_pkg     = get_package_share_directory('mr_peterbot_bringup')

    robot_description_path = os.path.join(description_pkg, 'urdf',
                                          'mr_peterbot.urdf.xacro')
    gz_bridge_params_path  = os.path.join(bringup_pkg, 'config',
                                          'ros_gz_bridge.yaml')

    # ===== LOAD URDF =====
    robot_description = xacro.process_file(robot_description_path).toxml()

    # ===== LAUNCH ARGUMENTS =====
    world_arg = DeclareLaunchArgument('world', default_value='empty.sdf',
                                      description='Gazebo world file')
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.1')
    R_arg = DeclareLaunchArgument('R', default_value='0.0')
    P_arg = DeclareLaunchArgument('P', default_value='0.0')
    Y_arg = DeclareLaunchArgument('Y', default_value='0.0')

    world_file = LaunchConfiguration('world')

    # ===== GAZEBO =====
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -v 4 ', world_file],
                          'on_exit_shutdown': 'true'}.items())

    # ===== NODES =====
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'mr_peterbot',
                   '-string', robot_description,
                   '-x', LaunchConfiguration('x'),
                   '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'),
                   '-R', LaunchConfiguration('R'),
                   '-P', LaunchConfiguration('P'),
                   '-Y', LaunchConfiguration('Y'),
                   '-allow_renaming', 'false'],
        output='screen')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
        output='screen')

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p',
                   f'config_file:={gz_bridge_params_path}'],
        output='screen')

    # ===== CONTROLLERS =====
    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'joint_state_broadcaster'],
        output='screen')

    forward_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'forward_velocity_controller'],
        output='screen')

    forward_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'forward_position_controller'],
        output='screen')

    # ===== LAUNCH DESCRIPTION =====
    return LaunchDescription([
        world_arg, x_arg, y_arg, z_arg, R_arg, P_arg, Y_arg,

        gazebo_launch,
        spawn_node,
        robot_state_publisher_node,
        gz_bridge_node,

        # Load controllers sequentially after spawning
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_node,
                on_exit=[joint_state_broadcaster])),

        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[forward_velocity_controller,
                         forward_position_controller])),
    ])
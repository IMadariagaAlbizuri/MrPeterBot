# Launch file to visualize Mr. PeterBot in Gazebo and RViz

# Libraries and dependencies
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

# Define the launch
def generate_launch_description():

    # Get package paths
    pkg = get_package_share_directory('mr_peterbot_description')
    urdf_file = os.path.join(pkg, 'urdf', 'mr_peterbot.urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', 'empty.sdf')

    # Load the Xacro file
    robot_description = xacro.process_file(urdf_file).toxml()

    return LaunchDescription([

        # Launch Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', world_file],
            output='screen'
        ),

        # Robot State Publisher NODE
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
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

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        ),
    ])
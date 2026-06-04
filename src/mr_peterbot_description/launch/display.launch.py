# Launch file to visualize Mr. PeterBot in RVIZ

# Libraries and dependencies
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

# Define the launch
def generate_launch_description():

    # Get package paths
    pkg = get_package_share_directory('mr_peterbot_description')
    urdf_file = os.path.join(pkg, 'urdf', 'mr_peterbot.urdf.xacro')

    # Load the Xacro file
    robot_description = xacro.process_file(urdf_file).toxml()

    return LaunchDescription([

        # Robot State Publisher NODE
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        # Joint State Publisher GUI NODE
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # RViz visualizer NODE
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        )
    ])
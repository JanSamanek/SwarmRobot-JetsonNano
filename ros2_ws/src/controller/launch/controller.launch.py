from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_file_name = 'robomaster.urdf'
    urdf_path = os.path.join(get_package_share_directory('urdf'), urdf_file_name)
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),

        Node(
            package='controller',
            executable='instructions_node',
            name='instructions_node',
        ),
    ])
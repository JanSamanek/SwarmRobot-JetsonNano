import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
      get_package_share_directory('tracker'),
      'config',
      'default_config.yaml'
      )
       
    return LaunchDescription([
        Node(
            package='tracker',
            namespace='tracker_ns',
            executable='tracker_node',
            name='tracker',
            parameters=[config]
        )
    ])
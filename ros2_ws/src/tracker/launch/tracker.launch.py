import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config = os.path.join(
      get_package_share_directory('tracker'),
      'config',
      'default_config.yaml'
      )
       
    params_file = LaunchConfiguration('params_file')
    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=config,
        description='Full path to the ROS2 parameters file with tracker configuration'
    )

    tracker_node = Node(
            package='tracker',
            executable='tracker_node',
            name='tracker',
            parameters=[params_file]
        )
    return LaunchDescription([
        declare_params_file_arg,
        tracker_node
    ])
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    segmentation_launch_file = PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('laser_segmentation'), 'launch/'),
            'segmentation.launch.py']) 

    rplidar_launch_file = PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('rplidar_ros'), 'launch/'),
      'rplidar_a2m8_launch.py'])
    
    tracker_launch_file = PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('tracker'), 'launch/'),
      'tracker.launch.py'])
    
    controller_launch_file = PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('controller'), 'launch/'),
      'controller.launch.py'])
    
    config_path = os.path.join(get_package_share_directory('swarm_robot_launch'), 'config', 'default_config.yaml')

    lidar_node = IncludeLaunchDescription(rplidar_launch_file)
    
    segmentation_node = IncludeLaunchDescription( segmentation_launch_file,
      launch_arguments={'params_file': config_path}.items())

    tracker_node = IncludeLaunchDescription(tracker_launch_file,
      launch_arguments={'params_file': config_path}.items()) 

    controller_node = IncludeLaunchDescription(controller_launch_file,
      launch_arguments={'params_file': config_path}.items())

    return LaunchDescription([
        lidar_node,
        segmentation_node,
        tracker_node,
        controller_node
    ])
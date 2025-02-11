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
      'rplidar_a3_launch.py'])
    
    tracker_launch_file = PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('tracker'), 'launch/'),
      'tracker.launch.py'])
    
    controller_share_dir = get_package_share_directory('controller')
    controller_config_path = os.path.join(controller_share_dir, 'config', 'controller_config.yaml')
    segmentation_config_path = os.path.join(controller_share_dir, 'config', 'segmentation_config.yaml')
    tracker_config_path = os.path.join(controller_share_dir, 'config', 'tracker_config.yaml')

    lidar_node = IncludeLaunchDescription(rplidar_launch_file)
    
    segmentation_node = IncludeLaunchDescription( segmentation_launch_file,
      launch_arguments={'params_file': segmentation_config_path}.items())

    tracker_node = IncludeLaunchDescription(tracker_launch_file,
      launch_arguments={'params_file': tracker_config_path}.items()) 

    controller_node = Node(
        package='controller',
        executable='controller_node',
        name='controller',
        parameters=[controller_config_path]
    )

    controller_node_delayed = TimerAction(
        period=5.0,  
        actions=[controller_node]
    )

    return LaunchDescription([
        lidar_node,
        segmentation_node,
        tracker_node,
        controller_node_delayed
    ])
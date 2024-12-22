import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    lidar_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('rplidar_ros'), 'launch/'),
      'view_rplidar_a3_launch.py']),
    )
    
    segmentation_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('laser_segmentation'), 'launch/'),
            'segmentation.launch.py']),
        launch_arguments={
                'distance_threshold': '0.15',
                'max_avg_distance_from_sensor': '1.5',
                'max_points_segment': '200',
                'max_segment_width': '0.4',
                'method_threshold': 'santos',
                'min_avg_distance_from_sensor': '0.0',
                'min_points_segment': '3',
                'min_segment_width': '0.05',
                'noise_reduction': '0.3',
                'restore_defaults': 'False',
                'scan_topic': 'scan',
                'segmentation_type': 'jump_distance_merge',
                'segments_topic': 'segments'}.items(),
    )

    tracker_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('tracker'), 'launch/'),
      'tracker.launch.py']),
    launch_arguments={'measurement_frequency': '10',
                      'kalman_filtering_enabled' : 'True'}.items(),
    )

    controller_node = Node(
        package='controller',
        executable='controller_node',
        name='controller',
        parameters=[os.path.join(
            get_package_share_directory('controller'),
            'config',
            'default_config.yaml')]
    )

    return LaunchDescription([
        lidar_node,
        segmentation_node,
        tracker_node,
        controller_node
    ])
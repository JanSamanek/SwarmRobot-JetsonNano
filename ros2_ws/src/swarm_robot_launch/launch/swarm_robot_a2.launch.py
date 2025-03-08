import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    
    urdf_file_name = 'swarmRobot.urdf'

    urdf=os.path.join(
        get_package_share_directory('swarm_robot_launch'),
        'urdf',
        urdf_file_name
    )

    with open(urdf, 'r') as infp:
      robot_desc = infp.read()
    
    robot_state_publisher_node = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
      output='screen',
      parameters=[{'robot_description': robot_desc}],
      arguments=[urdf]
      )
    
    odometry_node = Node(
        package='ros2_laser_scan_matcher',
        parameters=[{
            'base_frame':'base_link',
            'odom_frame':'odom',
            'laser_frame':'lidar',
            'publish_odom':'/odom',
            'publish_tf':False
        }],
        executable='laser_scan_matcher',
        name='odometry_publisher',
        output='screen',
    )
  
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
        robot_state_publisher_node,
        lidar_node,
        odometry_node,
        segmentation_node,
        tracker_node,
        controller_node
    ])
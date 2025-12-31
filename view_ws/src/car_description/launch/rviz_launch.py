import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('car_description')
    
    urdf_file = os.path.join(pkg_share, 'urdf', 'car.urdf')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'car_view.rviz')
    joy_config_file = os.path.join(pkg_share, 'config', 'joystick.yaml')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Madgwick Filter
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            parameters=[{'use_mag': False, 'fixed_frame': 'base_link', 'publish_tf': True}],
            remappings=[('/imu/data_raw', '/imu/data')]
        ),
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),
        # RViz2 with the LOADED CONFIG
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file] # <--- This loads your saved view!
        )
        # Joy Node (Reads physical controller)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'dev': '/dev/input/js0'}] # js0 is usually the first controller
        ),

        # Teleop Node (Converts Joy to Twist)
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[joy_config_file]
        ),

        # EKF Node (Estimates robot pose)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path]
        ),
    ])
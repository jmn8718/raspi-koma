import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Motor Control Node
    motor_node = Node(
        package='car_controller',
        executable='motors',
        name='motor_node',
        output='screen'
    )

    # 2. Dual Sonar Node (Left Front & Right Front)
    sonar_node = Node(
        package='car_controller', # Make sure this matches your sonar package name
        executable='sonar',
        name='sonar_node',
        output='screen'
    )

    # 3. Camera Node (Optimized for Pi 3B+ on Jazzy)
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_node',
        namespace='camera',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [320, 240],
            'pixel_format': 'YUYV',         # Changed from MJPG to YUYV
            'output_encoding': 'rgb8',      # ROS standard encoding
        }],
        remappings=[('/image_raw', '/camera/image_raw')]
    )

    imu_node = Node(
        package='car_controller',
        executable='imu',
        name='imu_node',
        parameters=[{
            'i2c_address': 0x68,
            'frequency': 10.0
        }]
    )

    # --- STATIC TRANSFORMS (TF) ---
    # x y z yaw pitch roll parent_frame child_frame
    
    # Camera: 12cm forward
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.12', '0.0', '0.05', '0.0', '0.0', '0.0', 'base_link', 'camera_link']
    )

    # Left Sonar: 10cm forward, 4.5cm left, angled 25 deg OUT (+0.436 rad)
    sonar_left_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.1', '0.045', '0.0', '0.436', '0.0', '0.0', 'base_link', 'sonar_left_link']
    )

    # Right Sonar: 10cm forward, 4.5cm right, angled 25 deg OUT (-0.436 rad)
    sonar_right_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.1', '-0.045', '0.0', '-0.436', '0.0', '0.0', 'base_link', 'sonar_right_link']
    )

    return LaunchDescription([
        motor_node,
        sonar_node,
        imu_node,
        camera_node,
        camera_tf,
        sonar_left_tf,
        sonar_right_tf
    ])

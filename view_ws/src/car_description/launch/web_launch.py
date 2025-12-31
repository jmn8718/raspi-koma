from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ROS Bridge
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket'
        ),

        # Web Video Server
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server'
        ),
    ])

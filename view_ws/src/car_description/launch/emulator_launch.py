import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('car_description')

    # Gazebo stack (robot_state_publisher, joints, imu, bridge, sim, spawn)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_path, 'launch', 'gazebo_launch.py')]),
    )

    # 7. RViz2 (Using your custom config)
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'raspikoma_view.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    # Ensure lingering Gazebo/bridge processes are terminated on shutdown.
    shutdown_handler = RegisterEventHandler(
        OnShutdown(on_shutdown=[
            ExecuteProcess(cmd=['pkill', '-f', 'ros_gz_bridge']),
            ExecuteProcess(cmd=['pkill', '-f', 'gz sim']),
        ])
    )

    return LaunchDescription([
        gazebo_launch,
        rviz,
        shutdown_handler,
    ])

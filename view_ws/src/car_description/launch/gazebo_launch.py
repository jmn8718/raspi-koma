import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. Path Setup
    pkg_path = get_package_share_directory('car_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'raspikoma.urdf')
    robot_description_config = xacro.process_file(urdf_file)

    # 2. Nodes
    # Robot State Publisher (Static TFs)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config.toxml(),
            'use_sim_time': True,
            'publish_frequency': 30.0
        }]
    )

    # Joint State Publisher
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description_config.toxml()
        }]
    )

    # 3. IMU Filter (Madgwick) - THIS MAKES THE ROBOT TILT
    # It takes /imu/data_raw and produces /imu/data with orientation
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'use_mag': False,
            'publish_tf': True,
            'world_frame': 'odom',
            'fixed_frame': 'odom',
            'base_frame': 'base_link' # Link the filter to the main chassis
        }]
    )

    # 4. Gazebo Sim Launcher
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        # -r runs automatically
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 5. Spawn Entity Node (The new way in Jazzy/Harmonic)
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'car', '-allow_renaming', 'true'],
    )

    # 6. The Bridge (Expanded to include IMU and Odometry)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # Try mapping the specific Gazebo topic to the ROS topic
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/ultrasonic/front_left@sensor_msgs/msg/Range[gz.msgs.Range',
            '/ultrasonic/front_right@sensor_msgs/msg/Range[gz.msgs.Range',
            '/model/car/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/car/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        remappings=[
            # If Gazebo is naming it one way and ROS another, force the link here
            ('/imu/data_raw', '/imu/data_raw'),
            ('/camera/image_raw', '/camera/image_raw'),
            # This ensures Gazebo's internal naming matches your ROS nodes
            ('/model/car/odometry', '/odom'),
            ('/model/car/tf', '/tf')
        ],
        output='screen'
    )
    
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher,
        imu_filter_node,
        bridge,
        gz_sim,
        gz_spawn_entity,
    ])

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_name = 'portamail_navigation'
    pkg_share = FindPackageShare(pkg_name)

    # --- Arguments ---
    # These flags allow you to selectively disable hardware during testing
    use_lidar_arg = DeclareLaunchArgument('use_lidar', default_value='true')
    use_teensy_arg = DeclareLaunchArgument('use_teensy', default_value='true')
    use_imu_arg = DeclareLaunchArgument('use_imu', default_value='true')
    use_ekf_arg = DeclareLaunchArgument('use_ekf', default_value='true')

    # --- 1. Teensy (Micro-ROS Agent) ---
    # Connects via USB Serial to the Teensy 4.0
    microros_agent = Node(
        condition=IfCondition(LaunchConfiguration('use_teensy')),
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='teensy_bridge',
        arguments=['serial', '--dev', '/dev/ttyACM0', '-b', '115200'],
        output='screen'
    )

    # --- 2. LiDAR (RPLIDAR A2) ---
    rplidar_node = Node(
        condition=IfCondition(LaunchConfiguration('use_lidar')),
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }]
    )

    # --- 3. IMU (BNO055) ---
    # Placeholder: Replace 'bno055' with the specific package you install
    imu_node = Node(
        condition=IfCondition(LaunchConfiguration('use_imu')),
        package='bno055', 
        executable='bno055', 
        name='imu_node',
        parameters=[{'uart_port': '/dev/ttyAMA0'}]
    )

    # --- 4. Static Transforms (URDF Replacement) ---
    # Lidar: 10cm forward (0.1), 10cm up (0.1)
    tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0.1', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
    )
    
    # IMU: Center of robot, 5cm up
    tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']
    )

    # --- 5. Sensor Fusion (EKF) ---
    ekf_config = PathJoinSubstitution([pkg_share, 'config', 'ekf.yaml'])
    
    ekf_node = Node(
        condition=IfCondition(LaunchConfiguration('use_ekf')),
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )

    return LaunchDescription([
        use_lidar_arg,
        use_teensy_arg,
        use_imu_arg,
        use_ekf_arg,
        microros_agent,
        rplidar_node,
        # imu_node, # Uncomment when package available
        tf_lidar,
        tf_imu,
        ekf_node
    ])
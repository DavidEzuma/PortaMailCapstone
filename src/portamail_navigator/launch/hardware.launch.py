import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_share = FindPackageShare('portamail_navigator')

    # --- URDF Configuration ---
    # This reads the URDF file and processes it (in case you switch to Xacro later)
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'portamail.urdf'])
    
    # Robot State Publisher node
    # This publishes the TF tree based on your SolidWorks model
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['cat ', urdf_file])}]
    )

    # --- Existing Arguments ---
    use_lidar_arg = DeclareLaunchArgument('use_lidar', default_value='true')
    use_teensy_arg = DeclareLaunchArgument('use_teensy', default_value='true')
    use_imu_arg = DeclareLaunchArgument('use_imu', default_value='true') 

    # --- 1. Micro-ROS Agent ---
    microros_agent = Node(
        condition=IfCondition(LaunchConfiguration('use_teensy')),
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='teensy_bridge',
        arguments=['serial', '--dev', '/dev/ttyACM0', '-b', '115200'],
        output='screen'
    )

    # --- 2. RPLIDAR A2 Driver ---
    rplidar_node = Node(
        condition=IfCondition(LaunchConfiguration('use_lidar')),
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser', # Ensure your URDF has a link named 'laser' or adjust this
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }]
    )

    # --- 3. IMU Driver ---
    imu_node = Node(
        condition=IfCondition(LaunchConfiguration('use_imu')),
        package='bno055', 
        executable='bno055', 
        name='imu_node',
        parameters=[{'uart_port': '/dev/ttyAMA0'}]
    )

    # --- 4. Sensor Fusion (EKF) ---
    ekf_config = PathJoinSubstitution([pkg_share, 'config', 'ekf.yaml'])
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )

    # --- NOTE ON STATIC TRANSFORMS ---
    # We REMOVED the manual static_tf_broadcaster for Lidar/IMU.
    # We assume your URDF links 'base_link' -> 'laser' and 'base_link' -> 'imu_link'.
    # If your URDF does NOT include the sensors, you must re-add the static TFs here.

    return LaunchDescription([
        use_lidar_arg,
        use_teensy_arg,
        use_imu_arg,
        robot_state_publisher, # <--- The new critical piece
        microros_agent,
        rplidar_node,
        ekf_node
    ])
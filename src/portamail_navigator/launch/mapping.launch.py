import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    pkg_share = FindPackageShare('portamail_navigator')

    # --- Arguments ---
    use_mock_driver_arg = DeclareLaunchArgument(
        'use_mock_driver', default_value='false',
        description='Use Mock Driver instead of Real Hardware')

    use_real_lidar_arg = DeclareLaunchArgument(
        'use_real_lidar', default_value='true',
        description='Launch physical RPLIDAR driver')

    # --- 1. Hardware Stack ---
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'hardware.launch.py'])
        ]),
        launch_arguments={
            'use_lidar': LaunchConfiguration('use_real_lidar'),
            'use_teensy': 'false', 
            'use_imu': 'false'
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('use_mock_driver'))
    )

    # Simulation Fallback (Mock Driver)
    # Note: We rely on hardware.launch (below) or a separate include to provide URDF if strictly mocking
    # For hybrid mode, hardware_launch provides the URDF state publisher.
    mock_driver_node = Node(
        condition=IfCondition(LaunchConfiguration('use_mock_driver')),
        package='portamail_navigator',
        executable='mock_driver',
        name='mock_driver',
        output='screen'
    )
    
    # If running PURE mock (no hardware launch), we need to launch state publisher separately.
    # But usually, we run hardware.launch with use_teensy:=false to get the URDF.
    # Let's ensure hardware launch runs even in mock mode, just disabling specific sensors.
    hybrid_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'hardware.launch.py'])
        ]),
        launch_arguments={
            'use_lidar': LaunchConfiguration('use_real_lidar'),
            'use_teensy': 'false',
            'use_imu': 'false'
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_mock_driver'))
    )

    # --- 2. Joystick Control ---
    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'joystick.launch.py'])
        ]),
        launch_arguments={'joy_dev': '/dev/input/js0'}.items()
    )

    # --- 3. SLAM Toolbox ---
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'scan_topic': '/scan',
            'mode': 'mapping'
        }]
    )

    # --- 4. Foxglove Bridge (CRITICAL FOR VISUALIZATION) ---
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'send_buffer_limit': 10000000
        }]
    )

    return LaunchDescription([
        use_mock_driver_arg,
        use_real_lidar_arg,
        hardware_launch,
        hybrid_hardware_launch,
        mock_driver_node,
        joystick_launch,
        slam_node,
        foxglove_bridge
    ])
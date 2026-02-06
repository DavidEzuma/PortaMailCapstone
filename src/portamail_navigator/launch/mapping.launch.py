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
        'use_mock_driver', default_value='true',  # Changed to true for testing
        description='Use Mock Driver instead of Real Hardware')

    use_real_lidar_arg = DeclareLaunchArgument(
        'use_real_lidar', default_value='false',  # Changed to false for pure simulation
        description='Launch physical RPLIDAR driver')

    # --- 1. Hardware Stack (provides URDF and robot_state_publisher) ---
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'hardware.launch.py'])
        ]),
        launch_arguments={
            'use_lidar': LaunchConfiguration('use_real_lidar'),
            'use_teensy': 'false',  # Always false when using mock
            'use_imu': 'false'
        }.items()
    )

    # --- 2. Mock Driver (provides /odom topic and TF) ---
    mock_driver_node = Node(
        condition=IfCondition(LaunchConfiguration('use_mock_driver')),
        package='portamail_navigator',
        executable='mock_driver',
        name='mock_driver',
        output='screen'
    )

    # --- 3. Joystick Control ---
    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'joystick.launch.py'])
        ]),
        launch_arguments={'joy_dev': '/dev/input/js0'}.items()
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
        }],
        output='screen'
    )

    return LaunchDescription([
        use_mock_driver_arg,
        use_real_lidar_arg,
        hardware_launch,
        mock_driver_node,
        joystick_launch,
        foxglove_bridge
    ])
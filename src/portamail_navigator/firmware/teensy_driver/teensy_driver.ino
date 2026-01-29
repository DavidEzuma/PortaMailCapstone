import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    pkg_share = FindPackageShare('portamail_navigation')

    # --- Arguments ---
    # Default to using Real Hardware for mapping
    use_mock_driver_arg = DeclareLaunchArgument(
        'use_mock_driver', default_value='false',
        description='Use Mock Driver instead of Real Hardware')

    # --- 1. Hardware Stack ---
    # If using Real Hardware, launch the hardware.launch.py file
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'hardware.launch.py'])
        ]),
        condition=UnlessCondition(LaunchConfiguration('use_mock_driver'))
    )

    # If using Mock Driver (Simulation), launch the C++ node + Static TF
    mock_driver_node = Node(
        condition=IfCondition(LaunchConfiguration('use_mock_driver')),
        package='portamail_navigation',
        executable='mock_driver',
        name='mock_driver',
        output='screen'
    )
    
    # We still need transforms even in Mock mode
    mock_tf_lidar = Node(
        condition=IfCondition(LaunchConfiguration('use_mock_driver')),
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0.1', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
    )

    # --- 2. Joystick Control ---
    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'joystick.launch.py'])
        ]),
        launch_arguments={'joy_dev': '/dev/input/js0'}.items()
    )

    # --- 3. SLAM Toolbox ---
    # Using 'online_async' mode for live mapping
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

    return LaunchDescription([
        use_mock_driver_arg,
        hardware_launch,
        mock_driver_node,
        mock_tf_lidar,
        joystick_launch,
        slam_node
    ])
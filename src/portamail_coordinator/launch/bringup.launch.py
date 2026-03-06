import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('portamail_coordinator')

    # Path to locations.yaml
    default_locations_file = PathJoinSubstitution([pkg_share, 'config', 'locations.yaml'])

    # --- Arguments ---
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='navigation',
        description='System mode: "mapping" or "navigation"'
    )

    loc_arg = DeclareLaunchArgument(
        'locations_file',
        default_value=default_locations_file,
        description='Path to the locations YAML file'
    )

    lcd_url_arg = DeclareLaunchArgument(
        'lcd_url',
        default_value='http://127.0.0.1:5050',
        description='Base URL of the LCD Flask server'
    )

    # --- Coordinator Node (C++) ---
    coordinator_node = Node(
        package='portamail_coordinator',
        executable='coordinator',
        name='navigation_coordinator',
        output='screen',
        parameters=[{
            'start_mode': LaunchConfiguration('mode'),
            'locations_file': LaunchConfiguration('locations_file'),
            'map_save_path': '/home/ubuntu/PortaMailCapstone/maps'
        }]
    )

    # --- LCD Bridge Node (Python) ---
    # Polls the touchscreen LCD API for events and bridges them to ROS topics.
    # In navigation mode: maps start_room1/2 → user_delivery_request, posts ARRIVED/DOCK_IDLE.
    # In mapping mode: forwards save_map events to coordinator.
    lcd_bridge_node = Node(
        package='portamail_coordinator',
        executable='lcd_bridge',
        name='lcd_bridge',
        output='screen',
        parameters=[{
            'lcd_url': LaunchConfiguration('lcd_url'),
            'poll_hz': 2.0,
            'ros_mode': LaunchConfiguration('mode'),
        }]
    )

    return LaunchDescription([
        mode_arg,
        loc_arg,
        lcd_url_arg,
        coordinator_node,
        lcd_bridge_node,
    ])

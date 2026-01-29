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

    # --- Coordinator Node ---
    coordinator_node = Node(
        package='portamail_coordinator',
        executable='coordinator',
        name='navigation_coordinator',
        output='screen',
        parameters=[{
            'start_mode': LaunchConfiguration('mode'),
            'locations_file': LaunchConfiguration('locations_file'),
            # Default save path for maps
            'map_save_path': '/home/ubuntu/PortaMailCapstone/maps' 
        }]
    )

    return LaunchDescription([
        mode_arg,
        loc_arg,
        coordinator_node
    ])
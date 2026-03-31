"""
bringup.launch.py — Coordinator + lcd_bridge for MAPPING mode.

Navigation mode now uses portamail_navigator/launch/navigation.launch.py
which includes the full hardware + AMCL + Nav2 + coordinator stack.
This file is kept for mapping mode only.

Persistent config paths (outside the install tree — not overwritten by colcon):
  locations_file : ~/PortaMailCapstone/config/locations.yaml
  maps_dir       : ~/PortaMailCapstone/maps/
  map_save_path  : ~/PortaMailCapstone/maps/   (passed to coordinator C++)
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    EnvironmentVariable,
)

_HOME = os.path.expanduser("~")
_DEFAULT_LOCATIONS_FILE = os.path.join(_HOME, "PortaMailCapstone", "config", "locations.yaml")
_DEFAULT_MAPS_DIR       = os.path.join(_HOME, "PortaMailCapstone", "maps")


def generate_launch_description():

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="mapping",
        description='System mode: "mapping" or "navigation"',
    )

    loc_arg = DeclareLaunchArgument(
        "locations_file",
        default_value=_DEFAULT_LOCATIONS_FILE,
        description="Persistent locations YAML (outside install tree)",
    )

    lcd_url_arg = DeclareLaunchArgument(
        "lcd_url",
        default_value="http://127.0.0.1:5050",
        description="Base URL of the LCD Flask server",
    )

    maps_dir_arg = DeclareLaunchArgument(
        "maps_dir",
        default_value=_DEFAULT_MAPS_DIR,
        description="Directory for map files",
    )

    # --- Coordinator (C++) ---
    coordinator_node = Node(
        package="portamail_coordinator",
        executable="coordinator",
        name="navigation_coordinator",
        output="screen",
        parameters=[{
            "start_mode":      LaunchConfiguration("mode"),
            "locations_file":  LaunchConfiguration("locations_file"),
            # map_save_path: use persistent path, not hardcoded /home/ubuntu
            "map_save_path":   _DEFAULT_MAPS_DIR,
        }],
    )

    # --- LCD Bridge (Python) ---
    lcd_bridge_node = Node(
        package="portamail_coordinator",
        executable="lcd_bridge",
        name="lcd_bridge",
        output="screen",
        parameters=[{
            "lcd_url":             LaunchConfiguration("lcd_url"),
            "poll_hz":             2.0,
            "ros_mode":            LaunchConfiguration("mode"),
            # locations_yaml_path: persistent path outside install tree so
            # colcon build does not overwrite waypoints saved during mapping.
            "locations_yaml_path": LaunchConfiguration("locations_file"),
            "maps_dir":            LaunchConfiguration("maps_dir"),
        }],
    )

    return LaunchDescription([
        mode_arg,
        loc_arg,
        lcd_url_arg,
        maps_dir_arg,
        coordinator_node,
        lcd_bridge_node,
    ])

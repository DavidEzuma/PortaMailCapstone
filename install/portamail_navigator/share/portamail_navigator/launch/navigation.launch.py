"""
navigation.launch.py — Full navigation stack for PortaMail.

Startup order (staged to let each layer settle before the next depends on it):

  t=0  Hardware:  robot_state_publisher, micro_ros_agent, sllidar_node, ekf_filter_node
  t+3  Localization: map_server + amcl  (lifecycle managed together)
  t+8  Nav2:      controller_server, smoother_server, planner_server,
                  behavior_server, velocity_smoother, bt_navigator
                  (lifecycle managed together)
  t+10 App layer: navigation_coordinator + lcd_bridge

The map file is auto-selected: the newest portamail_map_*.yaml in ~/PortaMailCapstone/maps/
is loaded.  If no map exists the launch aborts with a clear error — run mapping mode first.

Arguments:
  lcd_url          Base URL of the LCD Flask server (default: http://127.0.0.1:5050)
  mcu_port         Serial port for ESP32 micro-ROS agent
  locations_file   Persistent locations YAML (written by lcd_bridge during mapping)
  maps_dir         Directory to scan for map files
"""

import glob
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


# ---------------------------------------------------------------------------
# Persistent config paths (outside the install tree so colcon build doesn't
# overwrite them)
# ---------------------------------------------------------------------------
_HOME = os.path.expanduser("~")
_DEFAULT_MAPS_DIR = os.path.join(_HOME, "PortaMailCapstone", "maps")
_DEFAULT_LOCATIONS_FILE = os.path.join(_HOME, "PortaMailCapstone", "config", "locations.yaml")


def _resolve_map(context):
    """OpaqueFunction: find newest portamail_map_*.yaml in maps_dir."""
    maps_dir = LaunchConfiguration("maps_dir").perform(context)
    pattern = os.path.join(maps_dir, "portamail_map_*.yaml")
    candidates = sorted(glob.glob(pattern))  # ISO timestamp suffix sorts lexicographically
    if not candidates:
        raise RuntimeError(
            f"[navigation.launch] No map files found in {maps_dir!r}.\n"
            "Run mapping mode first and save a map before starting navigation."
        )
    newest = candidates[-1]
    import launch
    launch.logging.get_logger("navigation.launch").info(
        f"Loading map: {newest}"
    )
    return [launch.actions.SetLaunchConfiguration("resolved_map_yaml", newest)]


def generate_launch_description():
    nav_pkg = FindPackageShare("portamail_navigator")
    coord_pkg = FindPackageShare("portamail_coordinator")

    # -----------------------------------------------------------------------
    # Arguments
    # -----------------------------------------------------------------------
    lcd_url_arg = DeclareLaunchArgument(
        "lcd_url",
        default_value="http://127.0.0.1:5050",
        description="Base URL of the LCD Flask server",
    )

    mcu_port_arg = DeclareLaunchArgument(
        "mcu_port",
        default_value=(
            "/dev/serial/by-id/"
            "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
        ),
        description="Serial port for the ESP32 micro-ROS agent",
    )

    maps_dir_arg = DeclareLaunchArgument(
        "maps_dir",
        default_value=_DEFAULT_MAPS_DIR,
        description="Directory containing portamail_map_*.yaml map files",
    )

    locations_file_arg = DeclareLaunchArgument(
        "locations_file",
        default_value=_DEFAULT_LOCATIONS_FILE,
        description="Persistent locations YAML (written by lcd_bridge during mapping)",
    )

    # Placeholder — populated by _resolve_map OpaqueFunction
    resolved_map_arg = DeclareLaunchArgument(
        "resolved_map_yaml",
        default_value="",
        description="(internal) resolved map path, set by OpaqueFunction",
    )

    # -----------------------------------------------------------------------
    # Nav2 params file
    # -----------------------------------------------------------------------
    nav2_params = PathJoinSubstitution([nav_pkg, "config", "portamail_nav2_params.yaml"])

    # -----------------------------------------------------------------------
    # t=0  Hardware stack
    # -----------------------------------------------------------------------
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([nav_pkg, "launch", "hardware.launch.py"])]
        ),
        launch_arguments={
            "use_lidar":   "true",
            "use_mcu":     "true",
            "mcu_port":    LaunchConfiguration("mcu_port"),
            "use_imu":     "false",
            "use_ekf":     "true",
        }.items(),
    )

    # -----------------------------------------------------------------------
    # t+3  Localization: map_server + amcl  (lifecycle pair)
    # -----------------------------------------------------------------------
    map_server_node = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace="",
        output="screen",
        parameters=[
            nav2_params,
            {"yaml_filename": LaunchConfiguration("resolved_map_yaml")},
        ],
    )

    amcl_node = LifecycleNode(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        namespace="",
        output="screen",
        parameters=[nav2_params],
    )

    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[nav2_params],
    )

    localization_group = TimerAction(
        period=3.0,
        actions=[
            map_server_node,
            amcl_node,
            lifecycle_manager_localization,
        ],
    )

    # -----------------------------------------------------------------------
    # t+8  Navigation nodes (declared directly — no upstream navigation_launch.py
    #      so we skip route_server/docking_server/collision_monitor/waypoint_follower
    #      that PortaMail does not need and which add startup time + CPU load)
    # -----------------------------------------------------------------------
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params],
        remappings=[("cmd_vel", "cmd_vel_nav")],
    )

    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[nav2_params],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params],
    )

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[nav2_params],
        remappings=[
            ("cmd_vel",     "cmd_vel_nav"),
            ("cmd_vel_smoothed", "cmd_vel"),
        ],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params],
    )

    lifecycle_manager_navigation = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[nav2_params],
    )

    navigation_group = TimerAction(
        period=8.0,
        actions=[
            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            velocity_smoother,
            bt_navigator,
            lifecycle_manager_navigation,
        ],
    )

    # -----------------------------------------------------------------------
    # t+10  App layer: coordinator + lcd_bridge
    # -----------------------------------------------------------------------
    coordinator_node = Node(
        package="portamail_coordinator",
        executable="coordinator",
        name="navigation_coordinator",
        output="screen",
        parameters=[{
            "start_mode":      "navigation",
            "locations_file":  LaunchConfiguration("locations_file"),
            "map_save_path":   os.path.join(_HOME, "PortaMailCapstone", "maps"),
        }],
    )

    lcd_bridge_node = Node(
        package="portamail_coordinator",
        executable="lcd_bridge",
        name="lcd_bridge",
        output="screen",
        parameters=[{
            "lcd_url":             LaunchConfiguration("lcd_url"),
            "poll_hz":             2.0,
            "ros_mode":            "navigation",
            "locations_yaml_path": LaunchConfiguration("locations_file"),
            "maps_dir":            LaunchConfiguration("maps_dir"),
        }],
    )

    app_group = TimerAction(
        period=10.0,
        actions=[
            coordinator_node,
            lcd_bridge_node,
        ],
    )

    # -----------------------------------------------------------------------
    # Assemble
    # -----------------------------------------------------------------------
    return LaunchDescription([
        lcd_url_arg,
        mcu_port_arg,
        maps_dir_arg,
        locations_file_arg,
        resolved_map_arg,
        OpaqueFunction(function=_resolve_map),
        hardware_launch,
        localization_group,
        navigation_group,
        app_group,
    ])

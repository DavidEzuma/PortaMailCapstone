import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_share = FindPackageShare('portamail_navigator')

    # --- Arguments ---
    use_mock_driver_arg = DeclareLaunchArgument(
        'use_mock_driver', default_value='true',
        description='Use Mock Driver instead of Real Hardware')

    use_real_lidar_arg = DeclareLaunchArgument(
        'use_real_lidar', default_value='false',
        description='Launch physical RPLIDAR driver')

    autosave_enabled_arg = DeclareLaunchArgument(
        'autosave_enabled', default_value='true',
        description='Enable periodic map auto-save')

    autosave_interval_sec_arg = DeclareLaunchArgument(
        'autosave_interval_sec', default_value='30',
        description='Map auto-save period in seconds (0 = disabled)')

    map_output_dir_arg = DeclareLaunchArgument(
        'map_output_dir',
        default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'),
            'PortaMailCapstone',
            'maps',
        ]),
        description='Directory where auto-saved maps are stored')

    # --- 1. Hardware Stack (URDF, robot_state_publisher, LiDAR, EKF) ---
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'hardware.launch.py'])
        ]),
        launch_arguments={
            'use_lidar': LaunchConfiguration('use_real_lidar'),
            'use_teensy': 'false',
            'use_imu': 'false',
            # Disable EKF when mock_driver is active: mock_driver owns the
            # odom→base_link TF and running EKF in parallel causes TF conflicts
            # that make SLAM Toolbox drop scan messages.
            'use_ekf': 'false',
        }.items()
    )

    # --- 2. Mock Driver (provides /odom and odom->base_link TF) ---
    mock_driver_node = Node(
        condition=IfCondition(LaunchConfiguration('use_mock_driver')),
        package='portamail_navigator',
        executable='mock_driver',
        name='mock_driver',
        output='screen'
    )

    # --- 3. SLAM Toolbox (lifecycle node with auto configure/activate) ---
    slam_config = PathJoinSubstitution([pkg_share, 'config', 'slam.yaml'])

    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_config],
    )

    # Trigger configure 2 s after launch to let TF settle
    configure_slam_toolbox = TimerAction(
        period=2.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox_node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )
            ),
        ],
    )

    # Automatically activate once configure completes
    activate_slam_toolbox = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    # --- 4. Map Auto-Save Node ---
    autosave_config = PathJoinSubstitution([pkg_share, 'config', 'map_autosave.yaml'])

    map_autosave_node = Node(
        package='portamail_navigator',
        executable='map_autosave_node',
        name='map_autosave_node',
        output='screen',
        parameters=[
            autosave_config,
            {
                'output_directory': LaunchConfiguration('map_output_dir'),
                'autosave_interval_sec': LaunchConfiguration('autosave_interval_sec'),
            },
        ],
        condition=IfCondition(LaunchConfiguration('autosave_enabled')),
    )

    # --- 5. Joystick Control ---
    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'joystick.launch.py'])
        ]),
        launch_arguments={'joy_dev': '/dev/input/js0'}.items()
    )

    # --- 6. Foxglove Bridge (ws://<robot-ip>:8765) ---
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'send_buffer_limit': 10000000,  # 10 MB — desktop app doesn't need large buffer
            'publish_all_topics': True
        }],
        output='screen'
    )

    return LaunchDescription([
        use_mock_driver_arg,
        use_real_lidar_arg,
        autosave_enabled_arg,
        autosave_interval_sec_arg,
        map_output_dir_arg,
        hardware_launch,
        mock_driver_node,
        slam_toolbox_node,
        configure_slam_toolbox,
        activate_slam_toolbox,
        map_autosave_node,
        joystick_launch,
        foxglove_bridge,
    ])

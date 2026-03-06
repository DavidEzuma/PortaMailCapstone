from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare('portamail_navigator')
    
    # Paths
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'office.world'])
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'portamail_classic.urdf'])
    
    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # --- 1. GAZEBO SERVER (HEADLESS) ---
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_file],
        output='screen'
    )
    
    # --- 2. ROBOT STATE PUBLISHER ---
    robot_description = ParameterValue(Command(['cat ', urdf_file]), value_type=str)
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # --- 3. SPAWN ROBOT ---
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'portamail_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # --- 4. SLAM TOOLBOX ---
    slam_config = PathJoinSubstitution([pkg_share, 'config', 'slam.yaml'])
    
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # --- 5. JOYSTICK CONTROL ---
    coordinator_share = FindPackageShare('portamail_coordinator')
    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([coordinator_share, 'launch', 'joystick.launch.py'])
        ]),
        launch_arguments={'joy_dev': '/dev/input/js0'}.items()
    )
    
    # --- 6. FOXGLOVE BRIDGE ---
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'send_buffer_limit': 10000000,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        gazebo_server,
        robot_state_publisher,
        spawn_entity,
        slam_toolbox,
        joystick_launch,
        foxglove_bridge
    ])

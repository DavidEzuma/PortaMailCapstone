import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare('portamail_navigator')
    
    # Paths
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'office_world.world'])
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'portamail.urdf'])
    
    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # --- 1. GAZEBO HARMONIC SERVER (HEADLESS - NO GUI) ---
    # ROS 2 Jazzy uses gz sim (Gazebo Harmonic), not gzserver (Classic)
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_file],
        output='screen',
        shell=False
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
    
    # --- 3. SPAWN ROBOT IN GAZEBO HARMONIC ---
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'portamail_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # --- 4. ROS-GAZEBO BRIDGE (Critical for topic communication) ---
    # Bridge topics between ROS 2 and Gazebo Harmonic
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # --- 5. SLAM TOOLBOX (Online Async Mapping) ---
    slam_params_file = PathJoinSubstitution([pkg_share, 'config', 'slam_params.yaml'])
    
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # --- 6. JOYSTICK CONTROL (Xbox Controller) ---
    # Note: joystick.launch.py is in portamail_coordinator package
    coordinator_share = FindPackageShare('portamail_coordinator')
    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([coordinator_share, 'launch', 'joystick.launch.py'])
        ]),
        launch_arguments={'joy_dev': '/dev/input/js0'}.items()
    )
    
    # --- 7. FOXGLOVE BRIDGE (CRITICAL for headless operation) ---
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
        gazebo_server,  # NOTE: No GUI - headless mode
        robot_state_publisher,
        spawn_entity,
        gz_bridge,  # Bridge ROS 2 <-> Gazebo topics
        slam_toolbox,
        joystick_launch,
        foxglove_bridge
    ])

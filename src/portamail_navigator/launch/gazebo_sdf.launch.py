from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('portamail_navigator')
    
    # Paths
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'office_world.world'])
    sdf_model = PathJoinSubstitution([pkg_share, 'models', 'portamail_model.sdf'])
    
    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # --- 1. GAZEBO SERVER (HEADLESS) ---
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', '-v', '3', world_file],
        output='screen'
    )
    
    # --- 2. SPAWN ROBOT FROM SDF ---
    spawn_robot = ExecuteProcess(
        cmd=['gz', 'service', '-s', '/world/portamail_office/create',
             '--reqtype', 'gz.msgs.EntityFactory',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '1000',
             '--req', f'sdf_filename: "{sdf_model}", name: "portamail_robot", pose: {{position: {{z: 0.1}}}}'],
        output='screen'
    )
    
    # Register event handler to spawn robot after Gazebo starts
    spawn_after_gazebo = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_server,
            on_exit=[spawn_robot]
        )
    )
    
    # --- 3. ROS-GAZEBO BRIDGE ---
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        ],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # --- 4. SLAM TOOLBOX ---
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
    
    # --- 5. JOYSTICK CONTROL ---
    coordinator_share = FindPackageShare('portamail_coordinator')
    joystick_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }]
    )
    
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[{
            'axis_linear.x': 1,
            'axis_angular.yaw': 0,
            'scale_linear.x': 0.89,
            'scale_angular.yaw': 1.0,
            'enable_button': 7,
        }]
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
        gz_bridge,
        slam_toolbox,
        joystick_node,
        teleop_node,
        foxglove_bridge
    ])

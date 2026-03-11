import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue


SERIAL_PORT = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_3b22c239cde94c4e9a178cd7276563d2-if00-port0'
SERIAL_BAUD = 256000


def generate_launch_description():
    pkg_share = FindPackageShare('portamail_navigator')

    # Arguments to toggle hardware
    use_lidar_arg  = DeclareLaunchArgument('use_lidar',  default_value='true')
    use_teensy_arg = DeclareLaunchArgument('use_teensy', default_value='true')
    # use_imu: set to 'true' only when BNO055 is physically wired to the Teensy.
    # The Teensy reads BNO055 via I2C and publishes /imu/data over micro-ROS.
    # This flag only controls which EKF config is loaded (odom+IMU vs odom-only).
    # There is NO separate IMU ROS node on the Pi side.
    use_imu_arg    = DeclareLaunchArgument('use_imu',    default_value='false')

    # --- URDF / Robot State Publisher ---
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'portamail.urdf'])
    robot_description = ParameterValue(Command(['cat ', urdf_file]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 1. Micro-ROS Agent (Teensy 4.0 via USB → /dev/ttyACM0)
    # Built from source in ~/microros_ws (no apt/snap for arm64 Jazzy).
    # launch_portamail.sh sources ~/microros_ws/install/setup.bash so this
    # package is on the ROS path at launch time.
    microros_agent = Node(
        condition=IfCondition(LaunchConfiguration('use_teensy')),
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='teensy_bridge',
        arguments=['serial', '--dev', '/dev/ttyACM0', '-b', '115200'],
        output='screen'
    )

    # 2. RPLIDAR A2M12 Driver
    # scan_mode must be 'Sensitivity' (or 'Dense') — the A2M12 rejects 'Standard'
    # mode outright. usb_max_current_enable=1 in /boot/firmware/config.txt is also
    # required to supply enough USB current for the motor.
    # 2. SLLIDAR A2M12 Driver (sllidar_ros2 — Slamtec's updated ROS 2 SDK v2.1.0)
    # scan_mode must be 'Sensitivity': the A2M12 rejects 'Standard' mode.
    # Also requires usb_max_current_enable=1 in /boot/firmware/config.txt to
    # supply enough USB current for the motor.
    rplidar_node = Node(
        condition=IfCondition(LaunchConfiguration('use_lidar')),
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'serial_port':      SERIAL_PORT,
            'serial_baudrate':  SERIAL_BAUD,
            'frame_id':         'laser',
            'inverted':         False,
            'angle_compensate': True,
            'scan_mode':        'Sensitivity',
        }]
    )

    # 3. Sensor Fusion (EKF)
    def select_ekf_config(context):
        use_imu = LaunchConfiguration('use_imu').perform(context)
        config_file = 'ekf.yaml' if use_imu.lower() == 'true' else 'ekf_no_imu.yaml'
        ekf_config = PathJoinSubstitution([pkg_share, 'config', config_file])
        return [Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        )]

    ekf_launch = OpaqueFunction(function=select_ekf_config)

    return LaunchDescription([
        use_lidar_arg,
        use_teensy_arg,
        use_imu_arg,
        robot_state_publisher,
        microros_agent,
        rplidar_node,
        ekf_launch,
    ])

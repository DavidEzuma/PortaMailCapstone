from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Declare Argument for Device Path
    # Allows you to specify the device if it changes (e.g., js0 vs js1)
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Path to the joystick device (e.g., /dev/input/js0)'
    )

    return LaunchDescription([
        joy_dev_arg,

        # 2. Joy Node (Reads Bluetooth Input)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': LaunchConfiguration('joy_dev'),
                'deadzone': 0.1, # Increased slightly for wireless drift
                'autorepeat_rate': 20.0,
            }]
        ),

        # 3. Teleop Twist Joy (Converts to Velocity)
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[{
                'axis_linear.x': 1,  # Left Stick Vertical
                'axis_angular.yaw': 0, # Left Stick Horizontal
                'scale_linear.x': 0.89, # Max Speed (approx 2 MPH)
                'scale_angular.yaw': 1.0,
                'enable_button': 5, # Right Bumper (Must hold to move)
            }],
            # Remap directly to what the Teensy expects
            remappings=[('/cmd_vel', '/cmd_vel')]
        )
    ])
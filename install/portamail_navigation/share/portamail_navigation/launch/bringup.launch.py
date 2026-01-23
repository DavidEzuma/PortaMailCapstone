from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Declare arguments (variables you can change from CLI)
    # Example: ros2 launch portamail_navigation bringup.launch.py mode:=mapping
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='navigation',
        description='System mode: "mapping" or "navigation"'
    )

    # 2. Define the Coordinator Node
    coordinator_node = Node(
        package='portamail_navigation',
        executable='coordinator',
        name='navigation_coordinator',
        output='screen',
        parameters=[{
            'start_mode': LaunchConfiguration('mode'),
            # You can also set the default path here so you never have to type it again
            'locations_file': '/mnt/c/Users/David/OneDrive/Documents/Projects/PortaMailCapstone/locations.yaml'
        }]
    )

    # 3. Return the description
    return LaunchDescription([
        mode_arg,
        coordinator_node
    ])
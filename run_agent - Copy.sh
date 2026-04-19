#!/bin/bash
# Runs the Micro-ROS agent (serial transport, 115200 baud).
# Usage: ./run_agent.sh [device_path] (default: /dev/ttyUSB0)
#   ESP32 (CP2102 bridge) → /dev/ttyUSB0 when LiDAR is not connected
#   Use the stable by-id path if the enumeration order varies:
#     /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_...-if00-port0

DEVICE=${1:-/dev/ttyUSB0}
ARCH=$(uname -m)

if [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
    # Raspberry Pi 5 (BCM2712, arm64): use native agent built from source.
    # Docker humble image causes XRCE-DDS version mismatch with Jazzy nodes.
    # Build once with: micro_ros_setup create_agent_ws.sh && build_agent.sh
    MICROROS_WS="${HOME}/microros_ws"
    if [ ! -f "${MICROROS_WS}/install/setup.bash" ]; then
        echo "ERROR: Native micro-ROS agent not found at ${MICROROS_WS}."
        echo "Build it with:"
        echo "  mkdir -p ~/microros_ws/src && cd ~/microros_ws"
        echo "  git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup"
        echo "  source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash"
        echo "  ros2 run micro_ros_setup create_agent_ws.sh && ros2 run micro_ros_setup build_agent.sh"
        exit 1
    fi
    echo "Starting native micro-ROS agent (arm64/Jazzy) on $DEVICE..."
    source "${MICROROS_WS}/install/setup.bash"
    ros2 run micro_ros_agent micro_ros_agent serial --dev "$DEVICE" -b 115200
else
    # amd64 (laptop/desktop): use Docker Jazzy image
    echo "Starting micro-ROS agent (Docker/Jazzy) on $DEVICE..."
    docker run -it --rm --net=host --privileged -v /dev:/dev microros/micro-ros-agent:jazzy serial --dev "$DEVICE" -b 115200
fi

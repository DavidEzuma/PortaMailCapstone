#!/bin/bash
# Runs the Micro-ROS agent in Docker, exposing the USB device
# Usage: ./run_agent.sh [device_path] (default: /dev/ttyACM0)

DEVICE=${1:-/dev/ttyACM0}

echo "Starting Micro-ROS Agent on $DEVICE..."
# We use --net=host so the agent is visible to the ROS 2 network on the Pi
docker run -it --rm --net=host --privileged \
    -v /dev:/dev \
    microros/micro-ros-agent:humble \
    serial --dev $DEVICE -b 115200

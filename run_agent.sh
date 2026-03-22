#!/bin/bash
# Runs the Micro-ROS agent in Docker, exposing the USB device
# Usage: ./run_agent.sh [device_path] (default: /dev/ttyACM0)

DEVICE=${1:-/dev/ttyACM0}

# Select agent image: Jazzy on amd64 (laptop), Humble workaround on arm64 (Pi —
# no Jazzy arm64 image exists). micro_ros_arduino library is built for Jazzy;
# mismatching the agent version causes intermittent XRCE-DDS session drops.
ARCH=$(uname -m)
if [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
    AGENT_IMAGE="microros/micro-ros-agent:humble"
    echo "WARNING: Running humble agent on arm64 (no jazzy arm64 image available)"
else
    AGENT_IMAGE="microros/micro-ros-agent:jazzy"
fi

echo "Starting Micro-ROS Agent ($AGENT_IMAGE) on $DEVICE..."
# We use --net=host so the agent is visible to the ROS 2 network on the Pi
docker run -it --rm --net=host --privileged \
    -v /dev:/dev \
    $AGENT_IMAGE \
    serial --dev $DEVICE -b 115200

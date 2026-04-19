#!/bin/bash
# Launch Gazebo simulation HEADLESS with SLAM mapping
# Visualize through Foxglove at ws://[PI_IP]:8765

source ~/PortaMailCapstone/install/setup.bash

echo "=========================================="
echo "  Starting Headless Gazebo Simulation"
echo "=========================================="
echo ""
echo "Foxglove: ws://$(hostname -I | awk '{print $1}'):8765"
echo ""
echo "Press Ctrl+C to stop"
echo ""

ros2 launch portamail_navigator gazebo_mapping.launch.py

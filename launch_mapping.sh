#!/bin/bash
# Launch hardware mapping mode with SLAM
# Visualize through Foxglove at ws://[PI_IP]:8765

source ~/PortaMailCapstone/install/setup.bash

echo "=========================================="
echo "  Starting SLAM Mapping (Hardware)"
echo "=========================================="
echo ""
echo "Foxglove: ws://$(hostname -I | awk '{print $1}'):8765"
echo ""
echo "Drive around to build map, then save with:"
echo "  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \"{name: {data: 'MAP_NAME'}}\""
echo ""
echo "Press Ctrl+C to stop"
echo ""

ros2 launch portamail_navigator mapping.launch.py

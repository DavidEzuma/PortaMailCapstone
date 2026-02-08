#!/bin/bash

# Wait for Gazebo to be ready
echo "Waiting for Gazebo to start..."
sleep 3

# Get the SDF file path
SDF_FILE="$HOME/PortaMailCapstone/install/portamail_navigator/share/portamail_navigator/models/portamail_model.sdf"

if [ ! -f "$SDF_FILE" ]; then
    echo "ERROR: SDF file not found at $SDF_FILE"
    exit 1
fi

echo "Spawning robot from SDF..."

# Spawn using gz command
gz model --spawn-file="$SDF_FILE" \
         --model-name="portamail_robot" \
         --pose="0,0,0.1,0,0,0"

echo "Robot spawned successfully!"

#!/bin/bash

echo "=========================================="
echo "  Gazebo Classic 11 Setup (Headless)"
echo "=========================================="

# Function to check if package is installed
is_installed() {
    dpkg -s "$1" >/dev/null 2>&1
}

echo "[1/3] Installing Gazebo Classic 11..."

# Gazebo Classic packages for ROS 2 Jazzy
classic_deps=(
    "ros-jazzy-gazebo-ros-pkgs"
    "ros-jazzy-gazebo-plugins"
    "ros-jazzy-gazebo-ros"
    "gazebo"
)

to_install=()
for pkg in "${classic_deps[@]}"; do
    if ! is_installed "$pkg"; then
        echo "  + $pkg needed"
        to_install+=("$pkg")
    fi
done

if [ ${#to_install[@]} -ne 0 ]; then
    sudo apt update
    sudo apt install -y "${to_install[@]}"
else
    echo "  All packages already installed"
fi

echo ""
echo "[2/3] Verifying installation..."

if command -v gzserver &> /dev/null; then
    echo "  ✓ gzserver found: $(which gzserver)"
    gzserver --version
else
    echo "  ✗ gzserver not found!"
    exit 1
fi

echo ""
echo "[3/3] Creating headless launch helper..."

cat > ~/PortaMailCapstone/launch_gazebo_classic.sh << 'LAUNCHEOF'
#!/bin/bash
# Launch Gazebo Classic in headless mode

source ~/PortaMailCapstone/install/setup.bash

echo "=========================================="
echo "  Gazebo Classic Headless Simulation"
echo "=========================================="
echo ""
echo "Foxglove: ws://$(hostname -I | awk '{print $1}'):8765"
echo ""
echo "Press Ctrl+C to stop"
echo ""

ros2 launch portamail_navigator gazebo_classic_mapping.launch.py
LAUNCHEOF

chmod +x ~/PortaMailCapstone/launch_gazebo_classic.sh

echo ""
echo "=========================================="
echo "  Installation Complete!"
echo "=========================================="
echo ""
echo "Gazebo Classic 11 is ready for headless use"
echo "Next: Build the launch files and world"
echo ""

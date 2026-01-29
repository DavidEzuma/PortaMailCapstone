#!/bin/bash

# Stop on error
set -e

echo "==================================================="
echo "   PortaMail: Smart Setup & Build (ROS 2 Jazzy)  "
echo "==================================================="

# Function to check if an APT package is installed
is_installed() {
    dpkg -s "$1" >/dev/null 2>&1
}

# --- STEP 1: CHECK BASE ROS 2 INSTALLATION ---
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "[1/4] ROS 2 Jazzy found. Skipping base install."
else
    echo "[1/4] ROS 2 Jazzy NOT found. Installing base system..."
    
    sudo apt install -y software-properties-common
    sudo add-apt-repository -y universe

    sudo apt update && sudo apt install -y curl
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg --yes

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    # Install ROS Base (Headless) to avoid GUI conflicts on Pi
    sudo apt install ros-jazzy-ros-base -y
fi

# --- STEP 2: INSTALL PROJECT DEPENDENCIES ---
echo "[2/4] Checking project dependencies..."

# Removed: ros-jazzy-micro-ros-agent (Installing via Snap instead)
deps=(
    "ros-jazzy-navigation2"
    "ros-jazzy-nav2-bringup"
    "ros-jazzy-nav2-msgs"
    "ros-jazzy-slam-toolbox"
    "ros-jazzy-robot-localization"
    "ros-jazzy-rplidar-ros"
    "ros-jazzy-teleop-twist-joy"
    "ros-jazzy-tf2-ros"
    "ros-jazzy-tf2-geometry-msgs"
    "libyaml-cpp-dev"
    "python3-colcon-common-extensions"
)

# Build a list of ONLY missing packages
to_install=()
for pkg in "${deps[@]}"; do
    if is_installed "$pkg"; then
        echo "  - $pkg is already installed."
    else
        echo "  + $pkg is MISSING. Queuing for install."
        to_install+=("$pkg")
    fi
done

# Install only if there are missing packages
if [ ${#to_install[@]} -ne 0 ]; then
    echo "Installing missing dependencies: ${to_install[*]}"
    sudo apt update
    sudo apt install -y "${to_install[@]}"
else
    echo "All dependencies are satisfied. Skipping install."
fi

# --- STEP 2.5: INSTALL MICRO-ROS AGENT (VIA SNAP) ---
echo "[2.5/4] Checking Micro-ROS Agent..."
if snap list | grep -q "micro-ros-agent"; then
    echo "  - micro-ros-agent is already installed via Snap."
else
    echo "  + Installing micro-ros-agent via Snap..."
    sudo snap install micro-ros-agent
    # Connect necessary plugs for serial access
    sudo snap connect micro-ros-agent:serial-port
fi

# --- STEP 3: CONFIGURE ENVIRONMENT ---
echo "[3/4] Configuring environment..."

if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "Added ROS 2 to ~/.bashrc"
fi

source /opt/ros/jazzy/setup.bash

# --- STEP 4: BUILD BOTH PACKAGES ---
echo "[4/4] Building PortaMail Stack..."

# Clean old build artifacts to prevent caching errors
rm -rf build/ install/ log/

# Build Coordinator AND Navigation packages
colcon build --packages-select portamail_coordinator portamail_navigator

echo "==================================================="
echo "   SUCCESS! Build Complete.                      "
echo "==================================================="
echo "To start, run: source install/setup.bash"
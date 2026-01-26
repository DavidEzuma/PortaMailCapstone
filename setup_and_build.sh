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
    echo "[1/4] ROS 2 Jazzy found in /opt/ros/jazzy. Skipping base install."
else
    echo "[1/4] ROS 2 Jazzy NOT found. Installing base system..."
    
    # Enable Universe repo
    sudo apt install -y software-properties-common
    sudo add-apt-repository -y universe

    # Add ROS 2 GPG Key
    sudo apt update && sudo apt install -y curl
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg --yes

    # Add ROS 2 Repository
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    # Install ROS 2 Desktop
    sudo apt update
    # Install Bare Bones (No GUI tools)
    sudo apt install ros-jazzy-ros-base -y

    # Install only the specific robotics packages we need
    sudo apt install ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup -y
fi

echo "[1.5/4] Resolving package version conflicts..."
sudo apt update
sudo apt upgrade -y
sudo apt --fix-broken install -y

# --- STEP 2: CHECK & INSTALL PROJECT DEPENDENCIES ---
echo "[2/4] Checking project dependencies..."

# List of required packages
deps=(
    "ros-jazzy-navigation2"
    "ros-jazzy-nav2-bringup"
    "ros-jazzy-nav2-msgs"
    "ros-jazzy-slam-toolbox"
    "ros-jazzy-teleop-twist-joy"
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

# --- STEP 3: CONFIGURE ENVIRONMENT ---
echo "[3/4] Configuring environment..."

# Add ROS 2 to bashrc if not already there
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "Added ROS 2 to ~/.bashrc"
else
    echo "ROS 2 already in ~/.bashrc."
fi

# Source for this run
source /opt/ros/jazzy/setup.bash

# --- STEP 4: BUILD ---
echo "[4/4] Building PortaMail Navigation..."

# We always clean and build to ensure the code is fresh
rm -rf build/ install/ log/
colcon build --packages-select portamail_navigation

echo "==================================================="
echo "   SUCCESS! Build Complete.                      "
echo "==================================================="
echo "To start, run: source install/setup.bash"
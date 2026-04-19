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
    echo "[1/5] ROS 2 Jazzy found. Skipping base install."
else
    echo "[1/5] ROS 2 Jazzy NOT found. Installing base system..."
    
    sudo apt install -y software-properties-common curl
    sudo add-apt-repository -y universe

    sudo apt update && sudo apt install -y curl
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg --yes

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    # Install ROS Base (Headless) to avoid GUI conflicts on Pi
    sudo apt install ros-jazzy-ros-base -y
fi

# --- STEP 2: INSTALL PROJECT DEPENDENCIES ---
echo "[2/5] Checking project dependencies..."

# Hardware Drivers & Navigation Stack
deps=(
    "build-essential"
    "cmake"
    "ros-jazzy-navigation2"
    "ros-jazzy-nav2-bringup"
    "ros-jazzy-nav2-msgs"
    "ros-jazzy-slam-toolbox"
    "ros-jazzy-robot-localization"
    "ros-jazzy-rplidar-ros"
    "ros-jazzy-teleop-twist-joy"
    "ros-jazzy-joy"
    "ros-jazzy-tf2-ros"
    "ros-jazzy-tf2-geometry-msgs"
    "ros-jazzy-robot-state-publisher"
    "ros-jazzy-foxglove-bridge"
    "libyaml-cpp-dev"
    "python3-colcon-common-extensions"
)

# Build a list of ONLY missing packages
to_install=()
for pkg in "${deps[@]}"; do
    if ! is_installed "$pkg"; then
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

# --- STEP 3: SET UP MICRO-ROS AGENT (DOCKER ONLY) ---
echo "[3/5] Setting up Micro-ROS Agent (Docker)..."

# 1. Install Docker if missing
if ! command -v docker &> /dev/null; then
    echo "  + Installing Docker..."
    curl -fsSL https://get.docker.com | sh
    sudo usermod -aG docker $USER
    echo "  ! Docker installed. You may need to logout/login or reboot for group changes to take effect."
fi

# 2. Pull the Humble Agent Image (Compatible with Jazzy & Teensy)
echo "  + Pulling Micro-ROS Agent Docker image..."
sudo docker pull microros/micro-ros-agent:humble

# 3. Create helper script to run the agent
echo "  + Creating 'run_agent.sh' helper..."
cat > ~/PortaMailCapstone/run_agent.sh << 'EOF'
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
EOF
chmod +x ~/PortaMailCapstone/run_agent.sh

# --- STEP 4: BUILD ROS PACKAGES ---
echo "[4/5] Building PortaMail Stack..."

if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "Added ROS 2 to ~/.bashrc"
fi

source /opt/ros/jazzy/setup.bash

# Clean old build artifacts to ensure fresh build
rm -rf build/ install/ log/

# Build Coordinator AND Navigator packages
echo "Building 'portamail_navigator' and 'portamail_coordinator'..."
colcon build --packages-select portamail_coordinator portamail_navigator

echo "==================================================="
echo "   SUCCESS! Build Complete.                      "
echo "==================================================="
echo "IMPORTANT: You MUST run this command right now:"
echo "    source install/setup.bash"
echo ""
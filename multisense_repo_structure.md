# MultiSense S27 Camera Repository Structure

## Repository Layout
```
multisense-s27-setup/
├── README.md
├── setup.sh
├── scripts/
│   ├── network_setup.sh
│   ├── install_dependencies.sh
│   ├── build_workspace.sh
│   └── launch_camera.sh
├── config/
│   ├── multisense_params.yaml
│   └── rviz_config.rviz
├── launch/
│   └── custom_multisense_launch.py
├── docs/
│   ├── INSTALLATION.md
│   ├── TROUBLESHOOTING.md
│   └── NETWORK_SETUP.md
├── .gitignore
└── docker/
    ├── Dockerfile
    └── docker-compose.yml
```

## Key Files to Create

### 1. Main Setup Script (`setup.sh`)
```bash
#!/bin/bash

set -e

echo "=== MultiSense S27 Camera Setup ==="
echo "This script will set up the MultiSense S27 camera on your system"
echo ""

# Check if running as root
if [[ $EUID -eq 0 ]]; then
   echo "This script should not be run as root" 
   exit 1
fi

# Check Ubuntu version
if ! lsb_release -rs | grep -q "22.04"; then
    echo "Warning: This setup is designed for Ubuntu 22.04. Your version may not be compatible."
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Create workspace
WORKSPACE_DIR="$HOME/multisense_ws"
echo "Creating workspace at $WORKSPACE_DIR..."
mkdir -p $WORKSPACE_DIR/src

# Run setup scripts
echo "Running network setup..."
./scripts/network_setup.sh

echo "Installing dependencies..."
./scripts/install_dependencies.sh

echo "Building workspace..."
./scripts/build_workspace.sh

echo ""
echo "=== Setup Complete! ==="
echo "To launch the camera:"
echo "1. Connect the MultiSense S27 to your Ethernet port"
echo "2. Run: ./scripts/launch_camera.sh"
echo ""
echo "For troubleshooting, see docs/TROUBLESHOOTING.md"
```

### 2. Network Setup Script (`scripts/network_setup.sh`)
```bash
#!/bin/bash

set -e

echo "=== Network Configuration ==="

# Function to detect Ethernet interface
detect_ethernet_interface() {
    local interfaces=($(ip link show | grep -E "^[0-9]+: en" | cut -d: -f2 | tr -d ' '))
    
    if [ ${#interfaces[@]} -eq 0 ]; then
        echo "Error: No Ethernet interfaces found"
        exit 1
    elif [ ${#interfaces[@]} -eq 1 ]; then
        echo "${interfaces[0]}"
    else
        echo "Multiple Ethernet interfaces found:"
        for i in "${!interfaces[@]}"; do
            echo "$((i+1)). ${interfaces[$i]}"
        done
        read -p "Select interface (1-${#interfaces[@]}): " choice
        echo "${interfaces[$((choice-1))]}"
    fi
}

# Detect or create static connection
CAMERA_IP="10.66.171.21"
HOST_IP="10.66.171.200"
CONNECTION_NAME="multisense-static"

# Check if connection already exists
if nmcli con show "$CONNECTION_NAME" &>/dev/null; then
    echo "Connection '$CONNECTION_NAME' already exists"
    read -p "Reconfigure? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        nmcli con delete "$CONNECTION_NAME"
    else
        echo "Using existing connection"
        exit 0
    fi
fi

# Detect Ethernet interface
echo "Detecting Ethernet interface..."
ETH_INTERFACE=$(detect_ethernet_interface)
echo "Using interface: $ETH_INTERFACE"

# Create static connection
echo "Creating static network connection..."
nmcli con add type ethernet ifname "$ETH_INTERFACE" con-name "$CONNECTION_NAME"
nmcli con modify "$CONNECTION_NAME" ipv4.method manual
nmcli con modify "$CONNECTION_NAME" ipv4.addresses "$HOST_IP/24"
nmcli con modify "$CONNECTION_NAME" ipv4.never-default yes
nmcli con modify "$CONNECTION_NAME" connection.autoconnect yes

echo "Network configuration complete!"
echo "Host IP: $HOST_IP"
echo "Camera IP: $CAMERA_IP"

# Test connection
echo "Testing connection to camera..."
if nmcli con up "$CONNECTION_NAME" && ping -c 3 "$CAMERA_IP" &>/dev/null; then
    echo "✓ Camera connection successful!"
else
    echo "⚠ Warning: Cannot reach camera. Make sure it's connected and powered on."
fi
```

### 3. Dependencies Installation (`scripts/install_dependencies.sh`)
```bash
#!/bin/bash

set -e

echo "=== Installing Dependencies ==="

# Update package list
sudo apt update

# Install ROS2 Humble if not already installed
if ! command -v ros2 &> /dev/null; then
    echo "Installing ROS2 Humble..."
    
    # Add ROS2 repository
    sudo apt install software-properties-common -y
    sudo add-apt-repository universe -y
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    sudo apt update
    sudo apt install ros-humble-desktop -y
    
    # Add to bashrc if not already there
    if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    fi
    
    source /opt/ros/humble/setup.bash
else
    echo "ROS2 already installed"
fi

# Install required packages
echo "Installing ROS2 packages..."
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-image-geometry \
    ros-humble-cv-bridge \
    python3-colcon-common-extensions \
    git \
    build-essential

echo "Dependencies installation complete!"
```

### 4. Build Workspace Script (`scripts/build_workspace.sh`)
```bash
#!/bin/bash

set -e

WORKSPACE_DIR="$HOME/multisense_ws"

echo "=== Building MultiSense Workspace ==="

cd "$WORKSPACE_DIR"

# Source ROS2
source /opt/ros/humble/setup.bash

# Clone MultiSense repository if not exists
if [ ! -d "src/multisense_ros2" ]; then
    echo "Cloning MultiSense repository..."
    git clone https://github.com/carnegierobotics/multisense_ros2.git src/multisense_ros2
fi

# Initialize and update submodules
echo "Updating submodules..."
cd src/multisense_ros2
git submodule init
git submodule update
cd "$WORKSPACE_DIR"

# Install rosdep dependencies
echo "Installing rosdep dependencies..."
rosdep update
rosdep install --from-paths src --ignore-src -r -y || true

# Build packages
echo "Building MultiSense packages..."
colcon build --packages-select multisense_msgs multisense_lib multisense_ros

echo "Build complete!"
echo "Workspace ready at: $WORKSPACE_DIR"
```

### 5. Launch Script (`scripts/launch_camera.sh`)
```bash
#!/bin/bash

set -e

WORKSPACE_DIR="$HOME/multisense_ws"
CONNECTION_NAME="multisense-static"
CAMERA_IP="10.66.171.21"

echo "=== Launching MultiSense S27 Camera ==="

# Check if workspace exists
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "Error: Workspace not found at $WORKSPACE_DIR"
    echo "Please run ./setup.sh first"
    exit 1
fi

# Activate network connection
echo "Activating network connection..."
if ! nmcli con up "$CONNECTION_NAME" 2>/dev/null; then
    echo "Warning: Could not activate network connection"
    echo "Please check your network setup"
fi

# Test camera connectivity
echo "Testing camera connection..."
if ! ping -c 2 "$CAMERA_IP" &>/dev/null; then
    echo "Warning: Cannot reach camera at $CAMERA_IP"
    echo "Please ensure:"
    echo "1. Camera is connected to Ethernet port"
    echo "2. Camera is powered on"
    echo "3. Network configuration is correct"
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Launch the driver
cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Launching MultiSense driver..."
echo "Keep this terminal open. Use Ctrl+C to stop."
echo ""

ros2 launch multisense_ros multisense_launch.py
```

### 6. Main README.md
```markdown
# MultiSense S27 Camera Setup

Automated setup for Carnegie Robotics MultiSense S27 stereo camera on Ubuntu 22.04 with ROS2 Humble.

## Quick Start

1. **Clone this repository:**
   ```bash
   git clone https://github.com/yourusername/multisense-s27-setup.git
   cd multisense-s27-setup
   ```

2. **Run the setup script:**
   ```bash
   chmod +x setup.sh
   ./setup.sh
   ```

3. **Connect the camera** to your Ethernet port

4. **Launch the camera:**
   ```bash
   ./scripts/launch_camera.sh
   ```

## What This Does

- Configures static IP network connection (10.66.171.200/24)
- Installs ROS2 Humble and dependencies
- Creates a colcon workspace at `~/multisense_ws`
- Clones and builds the MultiSense ROS2 driver
- Provides launch scripts for easy camera operation

## Usage

### Basic Operation
- Start camera: `./scripts/launch_camera.sh`
- View topics: `ros2 topic list`
- Visualize data: `ros2 run rviz2 rviz2`

### Network Configuration
The camera uses IP `10.66.171.21` and your computer will be configured to `10.66.171.200`.

## Directory Structure

- `scripts/` - Setup and launch scripts
- `config/` - Configuration files
- `docs/` - Detailed documentation
- `launch/` - Custom launch files

## Requirements

- Ubuntu 22.04
- Ethernet port
- MultiSense S27 camera

## Documentation

- [Installation Guide](docs/INSTALLATION.md)
- [Network Setup](docs/NETWORK_SETUP.md)
- [Troubleshooting](docs/TROUBLESHOOTING.md)

## License

MIT License - see LICENSE file for details.
```

This structure provides:
- **Automated setup** for any Ubuntu 22.04 system
- **Network detection** and configuration
- **Dependency management** 
- **Easy launch scripts**
- **Comprehensive documentation**
- **Docker support** for containerized deployment
- **Configuration management** for different camera settings

Would you like me to create any specific files from this structure, or would you prefer to focus on a particular aspect like the Docker setup or custom launch configurations?
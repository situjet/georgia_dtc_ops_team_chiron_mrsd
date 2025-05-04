# DTC-MRSD Drone Autonomy & Gimbal Control Repository

## Introduction

This repository contains code for drone autonomy and gimbal control systems, primarily focusing on the humanflow ROS2 gimbal control system. This system allows users to control the drone's gimbal for target tracking and monitoring tasks.

## Repository Structure

- **AirStack/**: Air stack components for drone control
- **AirStack_GCS/**: Ground control station components
- **humanflow/**: ROS2-based gimbal control system
  - **ros2_ghadron_gimbal/**: Main gimbal control workspace
  - **docker/**: Docker configurations for the workspace
  - **mcap_data/**: Recorded data in MCAP format

## Installation Instructions

### Installing Docker and Docker Compose

1. Install Docker:

```bash
# Update package index
sudo apt-get update

# Install dependencies
sudo apt-get install apt-transport-https ca-certificates curl software-properties-common

# Add Docker's official GPG key
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

# Set up the stable repository
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"

# Install Docker Engine
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io

# Add current user to docker group to avoid using sudo
sudo usermod -aG docker $USER
```

2. Install Docker Compose:

```bash
# Download Docker Compose
sudo curl -L "https://github.com/docker/compose/releases/download/v2.18.1/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose

# Apply executable permissions
sudo chmod +x /usr/local/bin/docker-compose

# Test installation
docker-compose --version
```

### Clone the Repository

```bash
git clone https://github.com/your-organization/DTC-MRSD.git
cd DTC-MRSD
```

## Configuration Instructions

### Setting up the Docker Environment

#### Robot Component

1. Navigate to the AirStack directory:

```bash
cd AirStack
```

2. Build and start the robot Docker container:

```bash
# Build the robot container (HITL profile for Hardware-In-The-Loop)
docker compose --profile hitl build

# Start the robot container
docker compose --profile hitl up -d
```

#### Ground Control Station (GCS) Component

1. Navigate to the AirStack_GCS directory:

```bash
cd AirStack_GCS
```

2. Build and start the ground control station Docker container:

```bash
# Build the GCS container (HITL profile for Hardware-In-The-Loop)
docker compose --profile hitl build

# Start the GCS container
docker compose --profile hitl up -d
```

#### Gimbal Component

1. Navigate to the Docker configuration directory:

```bash
cd humanflow/ros2_ghadron_gimbal/dockers
```

2. Start the Docker containers:

```bash
docker compose -f docker-compose.yaml down
docker compose -f docker-compose.yaml up -d
```

### Permission Settings

Ensure the workspace has the correct permissions:

```bash
# Modify the permissions for the entire workspace
sudo chown -R $USER:$USER /home/dtc/humanflow/ros2_ghadron_gimbal

# Ensure installation directory has correct permissions
sudo chmod -R 755 /home/dtc/humanflow/ros2_ghadron_gimbal/install

# Especially for the site-packages directory
sudo chmod -R 755 /home/dtc/humanflow/ros2_ghadron_gimbal/install/detect_and_track/lib/python3.10/site-packages
```

## Operating Instructions

### Development Workflow

#### 1. SSH into the Remote Machine

```bash
ssh dtc@10.3.1.32
```

#### 2. Check Running Docker Containers

Ensure that the `[robot_l4t]` and `[gimbal]` containers are running:

```bash
docker ps
```

#### 3. Set Up Local Editing Environment

On your local machine, mount the remote directory using SSHFS:

```bash
sshfs dtc@10.3.1.32:/home/dtc/humanflow [path to your desired working directory]/mount
```

This allows you to edit files locally while the code runs on the remote machine.

#### 4. Connect to the Gimbal

```bash
# Enter the gimbal Docker container
docker exec -it ros2_ghadron_gimbal bash

# Launch the gimbal connection
ros2 launch gimbal_bringup gimbal_system.launch.py
```

**Note**: You should replace the radio ethernet with your own and connect to your laptop when working on it. The gimbal information goes to the Orin, and from the Orin to your machine.

#### 5. Build After Editing

When you've made changes to the code:

```bash
# Remove existing build, install, and log directories
./r_gimbal.sh

# Rebuild the selected packages
./b_gimbal.sh
```

#### 6. Run the Code

Run the code on the DTC host machine (not your local mount) using the same steps as in step 4.

### Common Commands

#### Docker Commands

```bash
# View running containers
docker ps

# Enter robot container
docker exec -it robot_l4t bash

# Enter ground control station container
docker exec -it ground-control-station-real bash

# Stop all containers
docker compose down
```

#### Build Commands
```bash
# Remove existing build artifacts
./r_gimbal.sh  # Removes install/inted_gimbal, install/payload, build/inted_gimbal, build/payload

# Build selected packages
./b_gimbal.sh  # Runs: colcon build --packages-select payload inted_gimbal
```

#### Gimbal Control Commands
```bash
# Launch the gimbal system
ros2 launch gimbal_bringup gimbal_system.launch.py

# Send control commands (pitch=0, roll=0, yaw=90)
ros2 topic pub /gimbal_angles geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: 90.0}"
```

#### Mode Control
```bash
# EO mode (Electro-Optical mode)
./eo_mode.sh

# IR mode (Infrared mode)
./ir_mode.sh

# Lock the gimbal
./lock.sh

# Point the gimbal downward
./pointdown.sh

# Return to home position
./homing.sh
```

## Additional Notes

- The pitch range of the gimbal is -90 (pointing down) to 90 (pointing up)
- The yaw range of the gimbal is -120 (left) to 120 (right)
- It's recommended to change directions incrementally (e.g., by 10 degrees each time) to avoid gimbal lock
- Main gimbal control path: `/home/dtc/humanflow/ros2_ghadron_gimbal/src/inted_gimbal/`
- The robot_l4t container uses NVIDIA runtime for GPU acceleration
- Both the robot_l4t and ground control station containers use host network mode for direct communication 
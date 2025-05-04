# DTC-MRSD Drone Autonomy & Gimbal Control Repository

This repository contains code for drone autonomy and gimbal control systems, primarily focusing on the humanflow ROS2 gimbal control system.

## Repository Structure

- **AirStack/**: Air stack components for drone control
- **AirStack_GCS/**: Ground control station components
- **humanflow/**: ROS2-based gimbal control system
  - **ros2_ghadron_gimbal/**: Main gimbal control workspace
  - **docker/**: Docker configurations for the workspace
  - **mcap_data/**: Recorded data in MCAP format

## Development Workflow

### 1. SSH into the Remote Machine

```bash
ssh dtc@10.3.1.32
```

### 2. Check Running Docker Containers

Ensure that the `[robot]` and `[gimbal]` containers are running:

```bash
docker ps
```

### 3. Set Up Local Editing Environment

On your local machine, mount the remote directory using SSHFS:

```bash
sshfs dtc@10.3.1.32:/home/dtc/humanflow [path to your desired working directory]/mount
```

This allows you to edit files locally while the code runs on the remote machine.

### 4. Connect to the Gimbal

```bash
# Navigate to home directory
cd ~

# Enter the gimbal Docker container
bash e_gim<Tab>  # This will autocomplete to the correct script

# Launch the gimbal connection
ros2 launch gim<Tab>b<Tab> g<Tab>  # This will autocomplete to the correct launch file
```

**Note**: You should replace the radio ethernet with your own and connect to your laptop when working on it. The gimbal information goes to the Orin, and from the Orin to your machine.

### 5. Build After Editing

When you've made changes to the code:

```bash
# Remove existing build, install, and log directories
./r_gimbal.sh

# Rebuild the selected packages
./b_gimbal.sh
```

### 6. Run the Code

Run the code on the DTC host machine (not your local mount) using the same steps as in step 4.

## Important Paths

- Main gimbal control: `/home/dtc/humanflow/ros2_ghadron_gimbal/src/inted_gimbal/`

## Common Commands

### Build Commands
```bash
# Remove existing build artifacts
./r_gimbal.sh  # Removes install/inted_gimbal, install/payload, build/inted_gimbal, build/payload

# Build selected packages
./b_gimbal.sh  # Runs: colcon build --packages-select payload inted_gimbal
```

### Gimbal Control Commands
```bash
# Launch the gimbal system
ros2 launch gimbal_bringup gimbal_system.launch.py

# Send control commands (pitch=0, roll=0, yaw=90)
ros2 topic pub /gimbal_angles geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: 90.0}"
```

## Additional Notes

- When working with the gimbal, ensure proper permissions are set for the workspace
- The pitch range is -90 (pointing down) to 90 (pointing up)
- The yaw range is -120 (left) to 120 (right)
- It's recommended to change directions incrementally (e.g., by 10 degrees each time) to avoid gimbal lock 
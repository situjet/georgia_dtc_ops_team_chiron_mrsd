# DTC-MRSD Unified Autonomy & Gimbal Control

## Overview

The system has been consolidated into a single (unified) ROS 2 Humble container that launches the flight autonomy stack (MAVROS + behavior governor + domain bridge) and provides the gimbal control nodes in the same workspace. Legacy multi-container (robot / GCS / humanflow gimbal) instructions have been deprecated and moved under `old/` for reference.

## Key Components (Current Runtime)

Launched automatically (via `robot.launch.xml`) inside the unified container:
- PX4 / MAVROS interface (`mavros_interface/px4.launch`)
- `behavior_governor` node (autonomy behavior management)
- `domain_bridge` (intra-domain/topic bridging per `domain_bridge.yaml`)

Available to run manually inside the container:
- `gimbal_control_node`, `gimbal_status_node`, `gimbal_angle_control_node` (package: `gimbal_control`)

Legacy / reference only:
- ROS1 teleoperation node (`operator/ros_ws/src/gimbal_teleop`) – not part of the current default runtime.

## Repository Structure (Active vs Legacy)

- `airstack/` – Active unified ROS2 workspace (`ros_ws/`)
  - `docker/Dockerfile.unified` – Build recipe for the unified image
  - `docker/run_unified.sh` – Helper script to start the unified runtime
  - `ros_ws/` – ROS2 workspace (packages: autonomy, behavior_governor, gimbal_control, robot_bringup, etc.)
- `operator/` – (Optional) legacy ROS1 teleop example
- `old/` – Historical multi-container stacks (no longer maintained)

## Prerequisites

Install Docker (and NVIDIA Container Toolkit if using Jetson / GPU). Docker Compose is not required for the unified flow.

## Quick Start

```bash
git clone <repo_url>
cd georgia_dtc_ops_team_chiron_mrsd
```

### 1. Build the Unified Image (if you need a fresh build)

The Dockerfile expects build context at `airstack/docker` so that `../ros_ws` is available for COPY.

```bash
cd airstack/docker
docker build -f Dockerfile.unified -t airstack-unified:latest .
```

### 2. Launch the Unified Container (recommended script)

From the repo root or from within `airstack/docker`:

```bash
./airstack/docker/run_unified.sh
```

This script:
1. Locates (without pulling) a suitable previously-built L4T AirStack image (or fails fast)
2. Starts container `airstack-unified` with:
   - Host networking
   - NVIDIA runtime (`--runtime nvidia`)
   - Mounted ROS workspace (`airstack/ros_ws` → `/root/ros_ws`)
   - SSH service enabled
   - Auto (re)build if `robot_bringup` not installed
3. Launches `robot_bringup/robot.launch.xml`
4. Streams logs to your terminal

If you want to force using the freshly built `airstack-unified:latest` image, temporarily tag / retag it to match the script’s discovery pattern or adjust the script (future enhancement: add explicit IMAGE override variable).

### 3. Verify Runtime

In another terminal:
```bash
docker ps --filter name=airstack-unified
docker logs -f airstack-unified | grep behavior_governor
```

You should see MAVROS connection attempts and the behavior governor starting.

### 4. Enter the Container

```bash
docker exec -it airstack-unified bash
source /opt/ros/humble/setup.bash
source /root/ros_ws/install/setup.bash
```

### 5. Run Gimbal Nodes (Manual)

```bash
ros2 run gimbal_control gimbal_status_node
ros2 run gimbal_control gimbal_control_node
ros2 run gimbal_control gimbal_angle_control_node
```

Example command to publish an angle vector (pitch=x, yaw=y, zoom=z placeholder):
```bash
ros2 topic pub /gimbal_angles geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: 90.0}" -r 1
```

### 6. Rebuilding After Code Changes

Because the workspace is bind-mounted, edit code on the host then inside the container:
```bash
cd /root/ros_ws
colcon build --symlink-install --merge-install
source install/setup.bash
```

If build artifacts get inconsistent:
```bash
rm -rf build install log
colcon build --symlink-install --merge-install
```

### 7. Stopping / Restarting

```bash
docker stop airstack-unified
./airstack/docker/run_unified.sh   # restart
```

## Environment Variables (Runtime)

Set before invoking `run_unified.sh` to override defaults:

| Variable        | Purpose                                    | Default |
|-----------------|--------------------------------------------|---------|
| `ROBOT_NAME`    | Logical robot name (sanitized to namespace)| `robot_1` |
| `ROBOT_NAMESPACE` | (Derived) ROS namespace                  | (sanitized `ROBOT_NAME`) |
| `ROS_DOMAIN_ID` | DDS domain isolation                       | `70` |

Example:
```bash
export ROBOT_NAME=field_unit_a
export ROS_DOMAIN_ID=42
./airstack/docker/run_unified.sh
```

## What the Entrypoint Launches

`airstack/ros_ws/entrypoint.sh` sources ROS, then launches:
```
ros2 launch robot_bringup robot.launch.xml
```
Which (see `robot_bringup/launch/robot.launch.xml`) includes:
- MAVROS PX4 interface
- behavior_governor (respawn)
- domain_bridge (respawn)

`gimbal_control` nodes are intentionally NOT auto-started—run only what you need.

## Legacy Components

The previous separation into:
- Robot (L4T) container
- Ground Control Station container
- Separate humanflow gimbal container

is retained only under `old/` for archival purposes. Do not mix those instructions with the unified flow.

## Gimbal Notes

- Pitch range: -90° (down) to +90° (up)
- Yaw range: -120° to +120°
- Prefer incremental movements to avoid saturating control

## Troubleshooting

| Symptom | Check |
|---------|-------|
| Container exits immediately | View `docker logs airstack-unified` for build or launch errors |
| MAVROS not connecting | Verify serial/device mappings (add device mounts if needed) |
| Nodes not found | Rebuild: remove `build install log` then `colcon build` |
| Gimbal topic absent | Ensure you started the desired `gimbal_control` node |

## Next Steps (Planned Enhancements)

- Add launch file to optionally auto-start gimbal nodes
- Allow IMAGE override in `run_unified.sh`
- Provide ROS1→ROS2 bridge or pure ROS2 teleop replacement

---
For historical documentation, consult files under `old/`. This README reflects the CURRENT supported unified workflow.
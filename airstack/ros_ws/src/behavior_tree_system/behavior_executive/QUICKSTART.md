# Quick Start Guide

## Installation in Any ROS 2 Workspace

### Method 1: Direct Copy
```bash
# Navigate to your ROS 2 workspace
cd ~/your_ros2_ws/src

# Clone just this standalone module
git clone -b standalone-behavior-executive --single-branch \
  git@github.com:LifGorg/TeamB_mrsd_24.git behavior_executive

# Build
cd ~/your_ros2_ws
colcon build --packages-select behavior_executive
```

### Method 2: As Git Submodule
```bash
cd ~/your_ros2_ws/src
git submodule add -b standalone-behavior-executive \
  git@github.com:LifGorg/TeamB_mrsd_24.git behavior_executive
```

## Dependencies

Install required dependencies:
```bash
sudo apt install ros-humble-mavros ros-humble-mavros-msgs
```

You'll also need:
- `behavior_tree` package
- `behavior_tree_msgs` package

## Quick Test

```bash
# Source your workspace
source ~/your_ros2_ws/install/setup.bash

# Run the behavior executive
ros2 run behavior_executive behavior_executive
```

## Integration

The module subscribes to `/behavior_tree_commands` for high-level commands and communicates with MAVROS for flight control.

See README.md for detailed architecture and customization options.

## What's Included

- ✅ Full behavior tree executive
- ✅ MAVROS integration layer
- ✅ Action handlers (geofence, navigation, search, survey)
- ✅ Transition logging
- ✅ Mission planning utilities
- ✅ No test dependencies
- ✅ Ready to use!

## File Structure

```
behavior_executive/
├── CMakeLists.txt          # Build configuration
├── package.xml             # ROS 2 package manifest
├── README.md               # Full documentation
├── QUICKSTART.md          # This file
├── include/               # All header files
│   └── behavior_executive/
└── src/                   # Implementation files
```

That's it! The module is ready to be integrated into any ROS 2 system with MAVROS.


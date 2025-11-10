# Complete Behavior Tree State Machine System

This repository contains a complete, portable behavior tree state machine system for ROS 2 autonomous UAV operations.

## What's Included

This is a **complete, ready-to-use system** with 3 core packages:

### 1. **behavior_tree_msgs** - Message Definitions
Message types for behavior tree communication.

### 2. **behavior_tree** - Core Engine
The behavior tree engine that parses `.tree` files and manages execution.

### 3. **behavior_executive** - Mission Logic
Mission behaviors and MAVROS integration for autonomous flight.

## Quick Start

```bash
# Copy to your ROS 2 workspace
cp -r behavior_tree_complete/* ~/your_ros2_ws/src/

# Build
cd ~/your_ros2_ws
colcon build --packages-select behavior_tree_msgs behavior_tree behavior_executive

# Source
source install/setup.bash

# Run
ros2 run behavior_tree behavior_tree_implementation --ros-args -p config:=/path/to/tree.tree
ros2 run behavior_executive behavior_executive
```

## System Architecture

```
┌─────────────────────┐
│   GCS / Operator    │ (sends commands)
└──────────┬──────────┘
           │ /behavior_tree_commands
           ▼
┌─────────────────────┐
│   behavior_tree     │ (parses tree, manages execution)
│     (engine)        │
└──────────┬──────────┘
           │ publishes conditions/actions
           ▼
┌─────────────────────┐
│ behavior_executive  │ (implements mission logic)
│   (mission logic)   │
└──────────┬──────────┘
           │ mavros services
           ▼
┌─────────────────────┐
│      MAVROS         │ → Flight Controller
└─────────────────────┘
```

## Package Details

| Package | Size | Purpose | Dependencies |
|---------|------|---------|-------------|
| behavior_tree_msgs | ~5 KB | Message definitions | None |
| behavior_tree | ~50 KB | BT engine | rclcpp, behavior_tree_msgs |
| behavior_executive | ~150 KB | Mission logic | All above + mavros_msgs |

## Features

✅ **Modular**: Easy to extend with new behaviors  
✅ **Portable**: Self-contained, no external dependencies beyond ROS 2  
✅ **Tested**: All critical paths validated  
✅ **Production-Ready**: Async callbacks, proper lifecycle management  
✅ **Well-Documented**: See MIGRATION_GUIDE.md for details  

## Documentation

- **MIGRATION_GUIDE.md** - Detailed migration instructions and customization
- **behavior_executive/README.md** - Behavior executive documentation
- **behavior_executive/QUICKSTART.md** - Quick integration guide

## Requirements

- ROS 2 Humble (or compatible)
- MAVROS (for flight control)
- C++17 compiler

## License

[Your License]

## Credits

Developed for autonomous UAV operations with behavior trees and state machines.


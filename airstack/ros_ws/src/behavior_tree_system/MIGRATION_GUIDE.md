# Complete Behavior Tree State Machine Migration Guide

## Overview

To migrate the full behavior tree state machine system to another codebase, you need **3 core packages** plus optional UI tools.

---

## Required Packages (Core System)

### 1. **behavior_tree_msgs** (Message Definitions)
**Location**: `AirStack/common/ros_packages/behavior_tree_msgs/`

**Files needed**:
```
behavior_tree_msgs/
├── CMakeLists.txt
├── package.xml
└── msg/
    ├── Active.msg              # Action/Condition active status
    ├── BehaviorTreeCommand.msg # Single command
    ├── BehaviorTreeCommands.msg # Command list
    └── Status.msg              # Status enum
```

**Dependencies**: None (pure message package)

**Purpose**: Defines the message types for commanding and monitoring the behavior tree.

---

### 2. **behavior_tree** (Core Engine)
**Location**: `AirStack/robot/ros_ws/src/autonomy/1_behavior/behavior_tree/`

**Files needed**:
```
behavior_tree/
├── CMakeLists.txt
├── package.xml
├── include/behavior_tree/
│   ├── behavior_tree.hpp              # Core BT classes (Node, Condition, Action)
│   └── behavior_tree_implementation.hpp # ROS 2 node implementation
├── src/
│   ├── behavior_tree.cpp
│   └── behavior_tree_implementation.cpp
└── config/
    └── drone.tree                     # Example tree configuration
```

**Dependencies**:
- `rclcpp`
- `behavior_tree_msgs`
- `std_msgs`

**Purpose**: Core behavior tree engine that parses `.tree` files and manages execution.

---

### 3. **behavior_executive** (Mission Logic)
**Location**: `AirStack/robot/ros_ws/src/autonomy/1_behavior/behavior_executive/`

**Already available as standalone module**: `standalone-behavior-executive` branch

**Files needed**:
```
behavior_executive/
├── CMakeLists.txt
├── package.xml
├── include/behavior_executive/
│   ├── behavior_executive.hpp          # Main executive
│   ├── mavros_adapter.hpp              # Flight controller interface
│   ├── mavros_adapter_interface.hpp
│   ├── transition_logger.hpp           # State logging
│   ├── action_handlers/                # Mission action implementations
│   │   ├── action_handler_interface.hpp
│   │   ├── geofence_mapping_handler.hpp
│   │   ├── navigate_to_waypoint_handler.hpp
│   │   ├── search_handler.hpp
│   │   └── survey_handler.hpp
│   └── mission_planning/               # Planning utilities
│       ├── geometry_utils.hpp
│       └── waypoint_generator.hpp
└── src/
    ├── behavior_executive.cpp
    ├── mavros_adapter.cpp
    ├── transition_logger.cpp
    ├── action_handlers/
    │   ├── geofence_mapping_handler.cpp
    │   ├── navigate_to_waypoint_handler.cpp
    │   ├── search_handler.cpp
    │   └── survey_handler.cpp
    └── mission_planning/
        ├── geometry_utils.cpp
        └── waypoint_generator.cpp
```

**Dependencies**:
- `rclcpp`
- `behavior_tree`
- `behavior_tree_msgs`
- `mavros_msgs`
- `sensor_msgs`
- `geometry_msgs`
- Optional: `airstack_msgs` (can be removed/replaced)

**Purpose**: Implements the actual mission behaviors and connects to MAVROS.

---

## Optional Packages (Development Tools)

### 4. **airstack_common** (Optional Utilities)
**Location**: `AirStack/common/ros_packages/airstack_common/`

**Only needed if**:
- You use the TF helpers (`tflib`)
- You use the visualization helpers (`vislib`)
- Your behavior_executive uses these utilities

**Can be replaced**: Most functionality can be replaced with standard ROS 2 libraries.

---

### 5. **rqt_behavior_tree** (Optional GUI)
**Location**: `AirStack/robot/ros_ws/src/autonomy/1_behavior/rqt_behavior_tree/`

**Purpose**: RQT plugin to visualize behavior tree execution in real-time.

**Optional**: Only needed for debugging/visualization.

---

### 6. **rqt_behavior_tree_command** (Optional GUI)
**Location**: `AirStack/robot/ros_ws/src/autonomy/1_behavior/rqt_behavior_tree_command/`

**Purpose**: RQT plugin to send commands to the behavior tree.

**Optional**: Commands can be sent programmatically or via command line.

---

## Migration Strategies

### Strategy 1: Minimal Migration (Recommended)

Copy only the 3 core packages:

```bash
# In your new workspace
cd ~/new_workspace/src

# Copy behavior_tree_msgs
cp -r /path/to/AirStack/common/ros_packages/behavior_tree_msgs .

# Copy behavior_tree
cp -r /path/to/AirStack/robot/ros_ws/src/autonomy/1_behavior/behavior_tree .

# Copy behavior_executive (or clone from standalone branch)
git clone -b standalone-behavior-executive --single-branch \
  git@github.com:LifGorg/TeamB_mrsd_24.git behavior_executive

# Build
cd ~/new_workspace
colcon build --packages-select behavior_tree_msgs behavior_tree behavior_executive
```

### Strategy 2: Full System Migration

Include UI tools for development:

```bash
cd ~/new_workspace/src

# Core packages (as above)
cp -r /path/to/AirStack/common/ros_packages/behavior_tree_msgs .
cp -r /path/to/AirStack/robot/ros_ws/src/autonomy/1_behavior/behavior_tree .
cp -r /path/to/AirStack/robot/ros_ws/src/autonomy/1_behavior/behavior_executive .

# Optional: UI tools
cp -r /path/to/AirStack/robot/ros_ws/src/autonomy/1_behavior/rqt_behavior_tree .
cp -r /path/to/AirStack/robot/ros_ws/src/autonomy/1_behavior/rqt_behavior_tree_command .

# Optional: Common utilities
cp -r /path/to/AirStack/common/ros_packages/airstack_common .

# Build all
cd ~/new_workspace
colcon build
```

---

## Package Dependency Graph

```
behavior_executive
    ├── depends on: behavior_tree
    ├── depends on: behavior_tree_msgs
    ├── depends on: mavros_msgs
    └── optionally: airstack_common

behavior_tree
    ├── depends on: behavior_tree_msgs
    └── depends on: rclcpp

behavior_tree_msgs
    └── (no dependencies - pure message package)

rqt_behavior_tree (optional)
    ├── depends on: behavior_tree_msgs
    └── depends on: rqt_gui

rqt_behavior_tree_command (optional)
    └── depends on: behavior_tree_msgs
```

---

## Quick Start After Migration

1. **Build the packages**:
```bash
cd ~/new_workspace
colcon build --packages-select behavior_tree_msgs behavior_tree behavior_executive
source install/setup.bash
```

2. **Create a tree file** (e.g., `my_mission.tree`):
```
? Root
    -> Arm Sequence
        (Arm Commanded)
        [Arm]
    -> Takeoff Sequence
        (Auto Takeoff Commanded)
        (Armed)
        [Request AutoTakeoff]
```

3. **Launch the system**:
```bash
# Terminal 1: Behavior tree engine
ros2 run behavior_tree behavior_tree_implementation --ros-args -p config:=/path/to/my_mission.tree

# Terminal 2: Behavior executive (mission logic)
ros2 run behavior_executive behavior_executive

# Terminal 3: Send commands
ros2 topic pub /behavior_tree_commands behavior_tree_msgs/msg/BehaviorTreeCommands ...
```

---

## Customization for New Codebase

### Remove airstack_msgs dependency:

1. Edit `behavior_executive/package.xml`:
   - Comment out or remove `<depend>airstack_msgs</depend>`

2. Edit `behavior_executive/CMakeLists.txt`:
   - Remove `airstack_msgs` from `find_package()`

3. Edit source files:
   - Replace any airstack-specific messages with standard ROS messages

### Adapt to different flight controller:

1. Implement custom `IMavrosAdapter`:
```cpp
class MyFlightControllerAdapter : public IMavrosAdapter {
    // Implement interface for your flight controller
};
```

2. Inject into BehaviorExecutive:
```cpp
auto adapter = std::make_shared<MyFlightControllerAdapter>();
auto executive = std::make_shared<BehaviorExecutive>(adapter);
```

---

## File Size Reference

- **behavior_tree_msgs**: ~5 KB (4 message files)
- **behavior_tree**: ~50 KB (core engine)
- **behavior_executive**: ~150 KB (all handlers and utilities)
- **Total core system**: ~205 KB

**All 3 core packages are lightweight and self-contained!**

---

## Summary

### Minimum Required (Core):
1. ✅ `behavior_tree_msgs` (messages)
2. ✅ `behavior_tree` (engine)
3. ✅ `behavior_executive` (logic)

### Optional (Development):
4. ⚪ `rqt_behavior_tree` (visualization)
5. ⚪ `rqt_behavior_tree_command` (GUI control)
6. ⚪ `airstack_common` (utilities)

### Recommended Approach:
Start with the 3 core packages from the `standalone-behavior-executive` branch for behavior_executive, and copy behavior_tree + behavior_tree_msgs from the original repo.


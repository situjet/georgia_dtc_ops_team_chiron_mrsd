# Behavior Executive - ROS 2 Behavior Tree Module

A plug-and-play ROS 2 package for autonomous drone behavior management using behavior trees.

## Overview

The Behavior Executive is a modular, extensible behavior tree implementation for autonomous UAV operations. It provides a clean separation of concerns through:

- **MAVROS Adapter**: Abstraction layer for flight controller communication
- **Action Handlers**: Modular handlers for mission actions (geofence mapping, navigation, search, survey)
- **Transition Logger**: Built-in logging for state machine debugging
- **Mission Planning Utilities**: Geometry and waypoint generation tools

## Features

✅ **Modular Architecture**: Easy to extend with new behaviors  
✅ **Testable**: Interface-based design for easy mocking and testing  
✅ **Production-Ready**: Includes async callback handling and proper lifecycle management  
✅ **Well-Logged**: Built-in transition logging for debugging state machines  
✅ **Safety-First**: Safety checks built into action handlers  

## Dependencies

- ROS 2 Humble
- MAVROS
- behavior_tree package
- behavior_tree_msgs package
- airstack_msgs package (for custom message types)

## Installation

### As a Standalone Package

1. Copy this directory to your ROS 2 workspace:
```bash
cp -r behavior_executive /path/to/your/ros2_ws/src/
```

2. Install dependencies:
```bash
cd /path/to/your/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build:
```bash
colcon build --packages-select behavior_executive
```

### As a Git Submodule

```bash
cd /path/to/your/ros2_ws/src
git submodule add <repository-url> behavior_executive
```

## Usage

### Basic Launch

```bash
ros2 run behavior_executive behavior_executive
```

### With Custom Behavior Tree

The behavior tree is loaded via behavior_tree_msgs commands. Connect your GCS or behavior planner to send commands via `/behavior_tree_commands` topic.

### Integration Example

```cpp
#include <behavior_executive/behavior_executive.hpp>

// The node automatically starts when instantiated
auto executive = std::make_shared<BehaviorExecutive>();
rclcpp::spin(executive);
```

## Architecture

```
behavior_executive/
├── include/behavior_executive/
│   ├── behavior_executive.hpp          # Main executive class
│   ├── mavros_adapter.hpp              # MAVROS interface implementation
│   ├── mavros_adapter_interface.hpp    # MAVROS abstraction
│   ├── transition_logger.hpp           # State transition logging
│   ├── action_handlers/                # Modular action handlers
│   │   ├── action_handler_interface.hpp
│   │   ├── geofence_mapping_handler.hpp
│   │   ├── navigate_to_waypoint_handler.hpp
│   │   ├── search_handler.hpp
│   │   └── survey_handler.hpp
│   └── mission_planning/               # Mission planning utilities
│       ├── geometry_utils.hpp
│       └── waypoint_generator.hpp
└── src/                                # Implementation files
```

## Extending the Module

### Adding a New Action Handler

1. Create a new handler class implementing `IActionHandler`:

```cpp
class MyCustomHandler : public IActionHandler {
public:
    void on_activated(bt::Action* action, const Context& ctx) override {
        // Your implementation
    }
    
    bool check_safety(const Context& ctx) override {
        // Safety checks
        return true;
    }
};
```

2. Instantiate in `BehaviorExecutive` constructor
3. Wire it to your behavior tree action

### Customizing MAVROS Integration

Implement `IMavrosAdapter` interface for custom flight controller integration or testing:

```cpp
class MyCustomAdapter : public IMavrosAdapter {
    // Implement interface methods
};

// Inject into BehaviorExecutive
auto custom_adapter = std::make_shared<MyCustomAdapter>();
auto executive = std::make_shared<BehaviorExecutive>(custom_adapter);
```

## Configuration

Transition logs are saved to `~/.ros/transitions/` by default. Configure via `TransitionLogger` constructor.

## Topics

### Subscribed
- `/behavior_tree_commands` - Behavior tree command messages
- `/mavros/state` - Vehicle state (armed, mode, etc.)
- `/mavros/global_position/global` - GPS position
- `/mavros/global_position/rel_alt` - Relative altitude
- `/selected_waypoint` - Target waypoint selection
- Additional mission-specific topics

### Published
- Behavior tree status topics (via behavior_tree package)

## Services Called
- `/mavros/cmd/arming` - Arm/disarm vehicle
- `/mavros/set_mode` - Change flight mode
- `/mavros/mission/push` - Upload mission waypoints
- `/mavros/mission/clear` - Clear mission waypoints

## Known Issues

- Requires behavior tree configuration to be loaded separately
- MAVROS services must be available before operation

## Contributing

This module is designed to be extended. Key extension points:
- Add new action handlers in `include/action_handlers/`
- Customize mission planning in `mission_planning/`
- Extend MAVROS adapter for additional services

## License

[Your License Here]

## Credits

Developed for autonomous UAV operations with PX4 and MAVROS.


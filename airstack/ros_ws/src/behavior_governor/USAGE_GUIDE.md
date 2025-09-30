# Behavior Governor 使用指南

## 概述

`behavior_governor` 是一个模块化的无人机行为控制系统，提供：
- **C++ 核心节点**：处理基础飞行操作（起飞、降落、航点等）
- **Python 扩展**：轻松添加复杂行为逻辑
- **ROS2 接口**：通过话题和服务进行通信

## 系统架构

```
Python 脚本 → behavior_governor/command → Behavior Governor (C++) → MAVROS → PX4
                ↑                                    ↓
            behavior_governor/status ←──────────────┘
```

## 快速开始

### 1. 启动系统

```bash
# 终端 1: 启动 MAVROS (连接到 PX4)
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14557

# 终端 2: 启动 Behavior Governor
source /home/triage/georgia_dtc_ops_team_chiron_mrsd/airstack/ros_ws/install/setup.bash
ros2 run behavior_governor behavior_governor_node

# 终端 3: 运行示例脚本
ros2 run behavior_governor example_behavior.py
```

### 2. 基本命令

通过发布到 `behavior_governor/command` 话题：

```python
# 基础命令
"arm"           # 解锁电机
"disarm"        # 锁定电机
"takeoff"       # 起飞
"land"          # 降落
"hold"          # 悬停
"rtl"           # 返航
"mission"       # 执行航点任务
"clear_waypoints"  # 清除航点
```

## 详细使用说明

### 1. 基础飞行操作

#### 简单起飞测试
```bash
# 使用内置测试脚本
ros2 run behavior_governor takeoff_tester.py

# 带参数运行
ros2 run behavior_governor takeoff_tester.py --arm-timeout 15.0 --ascend-timeout 60.0
```

#### 手动命令控制
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        self.cmd_pub = self.create_publisher(String, 'behavior_governor/command', 10)
        self.status_sub = self.create_subscription(String, 'behavior_governor/status', self.status_callback, 10)
        
    def status_callback(self, msg):
        self.get_logger().info(f'Status: {msg.data}')
        
    def arm_and_takeoff(self):
        # 解锁
        self.send_command('arm')
        time.sleep(2)
        # 起飞
        self.send_command('takeoff')
        
    def send_command(self, cmd):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'Sent: {cmd}')

if __name__ == '__main__':
    rclpy.init()
    controller = SimpleController()
    controller.arm_and_takeoff()
    rclpy.spin(controller)
```

### 2. 航点任务

#### 推送航点
```python
from behavior_governor.srv import PushWaypoints
from sensor_msgs.msg import NavSatFix

# 创建航点
waypoints = []
wp = NavSatFix()
wp.latitude = 40.4141646
wp.longitude = -79.9475044
wp.altitude = 10.0
waypoints.append(wp)

# 推送航点
request = PushWaypoints.Request()
request.waypoints = waypoints
request.hold_time = 3.0
request.acceptance_radius = 1.0
request.yaws = []

future = waypoint_client.call_async(request)
```

#### 完整航点任务示例
```bash
# 运行完整航点任务示例
ros2 run behavior_governor waypoint_mission_example.py
```

### 3. 状态监控

#### 监听状态
```python
from mavros_msgs.msg import State

def state_callback(self, msg):
    self.get_logger().info(f'Armed: {msg.armed}, Mode: {msg.mode}')

# 订阅状态
state_sub = self.create_subscription(State, 'behavior_governor/state', state_callback, 10)
```

#### 监听状态更新
```python
def status_callback(self, msg):
    self.get_logger().info(f'Governor Status: {msg.data}')

# 订阅状态更新
status_sub = self.create_subscription(String, 'behavior_governor/status', status_callback, 10)
```

## 高级功能

### 1. 自定义行为脚本

创建新的 Python 行为脚本：

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.msg import State

class CustomBehavior(Node):
    def __init__(self):
        super().__init__('custom_behavior')
        
        # 发布命令
        self.cmd_pub = self.create_publisher(String, 'behavior_governor/command', 10)
        
        # 监听状态
        self.state_sub = self.create_subscription(State, 'behavior_governor/state', self.state_callback, 10)
        self.status_sub = self.create_subscription(String, 'behavior_governor/status', self.status_callback, 10)
        
        # 定时器
        self.timer = self.create_timer(1.0, self.control_loop)
        
    def state_callback(self, msg):
        self.current_state = msg
        
    def status_callback(self, msg):
        self.get_logger().info(f'Status: {msg.data}')
        
    def control_loop(self):
        # 您的自定义逻辑
        if self.current_state and self.current_state.armed:
            # 执行复杂行为
            pass
            
    def send_command(self, cmd):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
```

### 2. 服务调用

#### 航点推送服务
```python
from behavior_governor.srv import PushWaypoints

# 创建服务客户端
waypoint_client = self.create_client(PushWaypoints, 'behavior_governor/push_waypoints')

# 等待服务可用
waypoint_client.wait_for_service()

# 调用服务
request = PushWaypoints.Request()
# ... 设置请求参数
future = waypoint_client.call_async(request)
```

## 故障排除

### 常见问题

1. **"MAVROS state not synchronized"**
   - 确保 MAVROS 已连接到 PX4
   - 检查 `mavros/state` 话题是否有数据

2. **"Arm rejected: vehicle appears to be airborne"**
   - 检查高度传感器数据
   - 确保飞机在地面

3. **"Mode switch failed"**
   - 检查 PX4 是否支持该模式
   - 确保满足模式切换条件

### 调试命令

```bash
# 查看话题
ros2 topic list | grep behavior_governor

# 监听状态
ros2 topic echo /behavior_governor/status

# 查看服务
ros2 service list | grep behavior_governor

# 检查节点
ros2 node list | grep behavior_governor
```

## 安全注意事项

1. **始终在安全环境中测试**
2. **确保有紧急停止机制**
3. **监控电池电量和信号强度**
4. **遵守当地法规**

## 示例脚本说明

- `example_behavior.py` - 基础示例
- `takeoff_tester.py` - 起飞测试（带详细监控）
- `waypoint_mission_example.py` - 完整航点任务
- `ladn_tester.py` - 降落测试
- `simple_takeoff_teester.py` - 简单起飞测试

## 扩展开发

要添加新的行为：

1. 在 `scripts/` 目录创建新的 Python 文件
2. 继承 `rclpy.Node`
3. 发布到 `behavior_governor/command`
4. 监听 `behavior_governor/status` 和 `behavior_governor/state`
5. 在 `CMakeLists.txt` 中添加安装配置

这样您就可以快速开发复杂的无人机行为，而无需修改 C++ 核心代码！

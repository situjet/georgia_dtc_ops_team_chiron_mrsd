#!/usr/bin/env python3
"""
Foxglove to BehaviorGovernor Bridge Node

This node acts as middleware between Foxglove Studio panels and the behavior_governor C++ node.
It listens to various command topics from Foxglove and translates them to behavior_governor commands.

Author: Georgia Tech DTC Ops Team
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State
from behavior_governor.srv import PushWaypoints
import json
import math


class FoxgloveBridge(Node):
    """Bridge node to connect Foxglove panels with behavior_governor"""
    
    def __init__(self):
        super().__init__('foxglove_bridge')
        
        # Publishers to behavior_governor
        self.behavior_cmd_pub = self.create_publisher(
            String, 
            'behavior_governor/command', 
            10
        )
        
        # Subscribers from Foxglove panels
        # Basic commands (arm, disarm, takeoff, land, etc.)
        self.foxglove_cmd_sub = self.create_subscription(
            String,
            'foxglove/command',
            self.handle_foxglove_command,
            10
        )
        
        # Arm/Disarm toggle
        self.arm_toggle_sub = self.create_subscription(
            Bool,
            'foxglove/arm_toggle',
            self.handle_arm_toggle,
            10
        )
        
        # Single waypoint command (go to location)
        self.goto_waypoint_sub = self.create_subscription(
            NavSatFix,
            'foxglove/goto_waypoint',
            self.handle_goto_waypoint,
            10
        )
        
        # Multiple waypoints (mission)
        self.mission_waypoints_sub = self.create_subscription(
            String,  # JSON string with waypoints array
            'foxglove/mission_waypoints',
            self.handle_mission_waypoints,
            10
        )
        
        # Click on map to set waypoint
        self.map_click_sub = self.create_subscription(
            PoseStamped,
            'foxglove/map_click',
            self.handle_map_click,
            10
        )
        
        # Subscriber to vehicle state for status feedback
        self.state_sub = self.create_subscription(
            State,
            'behavior_governor/state',
            self.handle_state_update,
            10
        )
        
        # Service client for pushing waypoints
        self.waypoint_client = self.create_client(
            PushWaypoints,
            'behavior_governor/push_waypoints'
        )
        
        # State tracking
        self.current_state = State()
        self.default_altitude = 10.0  # meters
        self.default_hold_time = 0.0  # seconds
        self.default_acceptance_radius = 0.5  # meters
        
        # Get parameters
        self.declare_parameter('default_altitude', 10.0)
        self.declare_parameter('default_hold_time', 0.0)
        self.declare_parameter('default_acceptance_radius', 0.5)
        
        self.default_altitude = self.get_parameter('default_altitude').value
        self.default_hold_time = self.get_parameter('default_hold_time').value
        self.default_acceptance_radius = self.get_parameter('default_acceptance_radius').value
        
        self.get_logger().info('FoxgloveBridge initialized')
        self.get_logger().info(f'Default altitude: {self.default_altitude}m')
        self.get_logger().info(f'Default hold time: {self.default_hold_time}s')
        self.get_logger().info(f'Default acceptance radius: {self.default_acceptance_radius}m')

    def handle_state_update(self, msg):
        """Update internal state tracking"""
        self.current_state = msg
        
    def handle_foxglove_command(self, msg):
        """Handle simple string commands from Foxglove panels"""
        command = msg.data.lower().strip()
        
        # Map Foxglove commands to behavior_governor commands
        command_map = {
            'arm': 'arm',
            'disarm': 'disarm',
            'takeoff': 'takeoff',
            'land': 'land',
            'hover': 'hold',
            'loiter': 'loiter',
            'hold': 'hold',
            'rtl': 'rtl',
            'return': 'rtl',
            'mission': 'mission',
            'start_mission': 'mission',
            'clear_waypoints': 'clear_waypoints',
            'clear_mission': 'clear_waypoints'
        }
        
        if command in command_map:
            # Send command to behavior_governor
            cmd_msg = String()
            cmd_msg.data = command_map[command]
            self.behavior_cmd_pub.publish(cmd_msg)
            self.get_logger().info(f'Sent command to behavior_governor: {cmd_msg.data}')
        else:
            self.get_logger().warn(f'Unknown Foxglove command: {command}')
    
    def handle_arm_toggle(self, msg):
        """Handle arm/disarm toggle from Foxglove"""
        cmd_msg = String()
        cmd_msg.data = 'arm' if msg.data else 'disarm'
        self.behavior_cmd_pub.publish(cmd_msg)
        self.get_logger().info(f'Sent {cmd_msg.data} command to behavior_governor')
    
    def handle_goto_waypoint(self, msg):
        """Handle single waypoint command from Foxglove"""
        # Create waypoint with default altitude if not specified
        waypoint = NavSatFix()
        waypoint.latitude = msg.latitude
        waypoint.longitude = msg.longitude
        waypoint.altitude = msg.altitude if msg.altitude != 0.0 else self.default_altitude
        
        # Push single waypoint using service
        self.push_waypoints_to_governor([waypoint])
    
    def handle_mission_waypoints(self, msg):
        """Handle multiple waypoints from Foxglove (JSON format)"""
        try:
            # Parse JSON waypoints
            data = json.loads(msg.data)
            waypoints = []
            yaws = []
            
            # Support different JSON formats
            if isinstance(data, list):
                # Array of waypoint objects
                for wp in data:
                    waypoint = NavSatFix()
                    waypoint.latitude = wp.get('lat', wp.get('latitude', 0.0))
                    waypoint.longitude = wp.get('lon', wp.get('longitude', 0.0))
                    waypoint.altitude = wp.get('alt', wp.get('altitude', self.default_altitude))
                    waypoints.append(waypoint)
                    
                    # Optional yaw angle
                    if 'yaw' in wp:
                        yaws.append(wp['yaw'])
            elif isinstance(data, dict):
                # Single object with waypoints array
                wp_list = data.get('waypoints', [])
                for wp in wp_list:
                    waypoint = NavSatFix()
                    waypoint.latitude = wp.get('lat', wp.get('latitude', 0.0))
                    waypoint.longitude = wp.get('lon', wp.get('longitude', 0.0))
                    waypoint.altitude = wp.get('alt', wp.get('altitude', self.default_altitude))
                    waypoints.append(waypoint)
                    
                    # Optional yaw angle
                    if 'yaw' in wp:
                        yaws.append(wp['yaw'])
            
            if waypoints:
                # Push waypoints using service
                hold_time = data.get('hold_time', self.default_hold_time) if isinstance(data, dict) else self.default_hold_time
                acceptance_radius = data.get('acceptance_radius', self.default_acceptance_radius) if isinstance(data, dict) else self.default_acceptance_radius
                
                self.push_waypoints_to_governor(
                    waypoints, 
                    hold_time=hold_time,
                    acceptance_radius=acceptance_radius,
                    yaws=yaws if yaws else []
                )
            else:
                self.get_logger().warn('No valid waypoints found in JSON data')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON waypoints: {e}')
        except Exception as e:
            self.get_logger().error(f'Error handling mission waypoints: {e}')
    
    def handle_map_click(self, msg):
        """Handle map click from Foxglove to set waypoint"""
        # Convert map click (PoseStamped) to waypoint
        # This assumes the map frame provides lat/lon in the position
        # You may need to transform coordinates based on your map setup
        
        waypoint = NavSatFix()
        # If using a projected coordinate system, you'll need to convert
        # For now, assuming position.x = longitude, position.y = latitude
        waypoint.latitude = msg.pose.position.y
        waypoint.longitude = msg.pose.position.x
        waypoint.altitude = msg.pose.position.z if msg.pose.position.z != 0.0 else self.default_altitude
        
        # Extract yaw from quaternion if needed
        # quaternion to euler conversion
        qw = msg.pose.orientation.w
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        
        # Calculate yaw (rotation around z-axis)
        yaw_rad = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        yaw_deg = math.degrees(yaw_rad)
        
        # Push waypoint with yaw
        self.push_waypoints_to_governor([waypoint], yaws=[yaw_deg])
    
    def push_waypoints_to_governor(self, waypoints, hold_time=None, acceptance_radius=None, yaws=None):
        """Push waypoints to behavior_governor using service"""
        if not waypoints:
            self.get_logger().warn('No waypoints to push')
            return
        
        # Wait for service to be available
        if not self.waypoint_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('behavior_governor waypoint service not available')
            return
        
        # Create service request
        request = PushWaypoints.Request()
        request.waypoints = waypoints
        request.hold_time = hold_time if hold_time is not None else self.default_hold_time
        request.acceptance_radius = acceptance_radius if acceptance_radius is not None else self.default_acceptance_radius
        
        # Add yaw angles if provided
        if yaws:
            request.yaws = yaws
        
        # Call service asynchronously
        future = self.waypoint_client.call_async(request)
        future.add_done_callback(self.waypoint_service_callback)
        
        self.get_logger().info(f'Pushing {len(waypoints)} waypoints to behavior_governor')
    
    def waypoint_service_callback(self, future):
        """Handle response from waypoint push service"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully pushed {response.waypoints_pushed} waypoints: {response.message}')
                
                # Optionally switch to mission mode after pushing waypoints
                if self.current_state.armed and response.waypoints_pushed > 0:
                    # Send mission command to start executing waypoints
                    cmd_msg = String()
                    cmd_msg.data = 'mission'
                    self.behavior_cmd_pub.publish(cmd_msg)
                    self.get_logger().info('Sent mission command to start waypoint execution')
            else:
                self.get_logger().error(f'Failed to push waypoints: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    bridge = FoxgloveBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

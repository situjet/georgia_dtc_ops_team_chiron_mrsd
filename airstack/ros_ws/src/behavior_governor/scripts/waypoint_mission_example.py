#!/usr/bin/env python3
"""
Example of using the behavior_governor waypoint push service.
This demonstrates how external nodes can push waypoints and execute missions.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from behavior_governor.srv import PushWaypoints
import time

class WaypointMissionExample(Node):
    def __init__(self):
        super().__init__('waypoint_mission_example')
        
        # Command publisher
        self.cmd_pub = self.create_publisher(String, 'behavior_governor/command', 10)
        
        # Status and state subscribers
        self.status_sub = self.create_subscription(
            String, 'behavior_governor/status', 
            self.status_callback, 10)
        
        self.state_sub = self.create_subscription(
            State, 'behavior_governor/state',
            self.state_callback, 10)
        
        # Waypoint push service client
        self.waypoint_client = self.create_client(PushWaypoints, 'behavior_governor/push_waypoints')
        
        self.current_state = None
        self.last_status = None
        
        self.get_logger().info('Waypoint mission example initialized')
        
    def state_callback(self, msg):
        """Track current vehicle state"""
        self.current_state = msg
        
    def status_callback(self, msg):
        """Receive command execution feedback"""
        self.last_status = msg.data
        self.get_logger().info(f'Status: {msg.data}')
        
    def send_command(self, cmd):
        """Send atomic command"""
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'Sent command: {cmd}')
        
    def wait_for_armed(self, timeout=10):
        """Wait for vehicle to be armed"""
        start = time.time()
        while time.time() - start < timeout:
            if self.current_state and self.current_state.armed:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False
        
    def wait_for_mode(self, target_mode, timeout=10):
        """Wait for specific flight mode"""
        start = time.time()
        while time.time() - start < timeout:
            if self.current_state and self.current_state.mode == target_mode:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False
        
    def push_test_waypoints(self):
        """Push a test waypoint mission"""
        # Wait for service to be available
        if not self.waypoint_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Waypoint push service not available')
            return False
            
        # Create test waypoints (square pattern)
        waypoints = []
        
        # Base coordinates (adjust for your test area)
        base_lat = 40.4141646
        base_lon = -79.9475044
        altitude = 10.0
        
        # Square pattern: 50m x 50m
        lat_offset = 0.00045  # ~50m north
        lon_offset = 0.00060  # ~50m east
        
        # Waypoint 1: Start
        wp1 = NavSatFix()
        wp1.latitude = base_lat
        wp1.longitude = base_lon
        wp1.altitude = altitude
        waypoints.append(wp1)
        
        # Waypoint 2: North
        wp2 = NavSatFix()
        wp2.latitude = base_lat + lat_offset
        wp2.longitude = base_lon
        wp2.altitude = altitude
        waypoints.append(wp2)
        
        # Waypoint 3: Northeast
        wp3 = NavSatFix()
        wp3.latitude = base_lat + lat_offset
        wp3.longitude = base_lon + lon_offset
        wp3.altitude = altitude
        waypoints.append(wp3)
        
        # Waypoint 4: East
        wp4 = NavSatFix()
        wp4.latitude = base_lat
        wp4.longitude = base_lon + lon_offset
        wp4.altitude = altitude
        waypoints.append(wp4)
        
        # Waypoint 5: Return to start
        wp5 = NavSatFix()
        wp5.latitude = base_lat
        wp5.longitude = base_lon
        wp5.altitude = altitude
        waypoints.append(wp5)
        
        # Create service request
        request = PushWaypoints.Request()
        request.waypoints = waypoints
        request.hold_time = 3.0  # Hold 3 seconds at each waypoint
        request.acceptance_radius = 1.0  # 1 meter acceptance radius
        request.yaws = []  # No specific yaw requirements
        
        self.get_logger().info(f'Pushing {len(waypoints)} waypoints...')
        
        # Call service
        future = self.waypoint_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully pushed {response.waypoints_pushed} waypoints')
                self.get_logger().info(f'Message: {response.message}')
                return True
            else:
                self.get_logger().error(f'Failed to push waypoints: {response.message}')
                return False
        else:
            self.get_logger().error('Service call failed')
            return False
            
    def execute_complete_mission(self):
        """Execute a complete waypoint mission"""
        self.get_logger().info('Starting complete waypoint mission...')
        
        # Step 1: Arm
        self.send_command('arm')
        if not self.wait_for_armed():
            self.get_logger().error('Failed to arm vehicle')
            return
        self.get_logger().info('Vehicle armed successfully')
        
        # Step 2: Takeoff
        time.sleep(1)
        self.send_command('takeoff')
        if not self.wait_for_mode('AUTO.TAKEOFF', timeout=15):
            self.get_logger().error('Failed to enter takeoff mode')
            return
        self.get_logger().info('Takeoff initiated')
        
        # Wait for takeoff to complete (switch to loiter)
        time.sleep(10)
        
        # Step 3: Clear any existing waypoints
        self.send_command('clear_waypoints')
        time.sleep(2)
        
        # Step 4: Push waypoints
        if not self.push_test_waypoints():
            self.get_logger().error('Failed to push waypoints')
            return
            
        # Step 5: Start mission
        time.sleep(2)
        self.send_command('mission')
        if not self.wait_for_mode('AUTO.MISSION', timeout=10):
            self.get_logger().error('Failed to enter mission mode')
            return
        self.get_logger().info('Mission started!')
        
        # Step 6: Monitor mission (in real scenario, monitor waypoint progress)
        self.get_logger().info('Mission executing... (monitor via status messages)')
        time.sleep(30)  # Let mission run for 30 seconds
        
        # Step 7: Return to launch
        self.send_command('rtl')
        self.get_logger().info('Returning to launch')
        time.sleep(20)
        
        # Step 8: Disarm (after landing)
        self.send_command('disarm')
        self.get_logger().info('Mission completed!')

def main():
    rclpy.init()
    
    mission_node = WaypointMissionExample()
    
    # Wait a bit for connections
    time.sleep(2)
    
    try:
        # Execute the complete mission
        mission_node.execute_complete_mission()
        
    except KeyboardInterrupt:
        mission_node.get_logger().info('Mission interrupted by user')
    
    mission_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

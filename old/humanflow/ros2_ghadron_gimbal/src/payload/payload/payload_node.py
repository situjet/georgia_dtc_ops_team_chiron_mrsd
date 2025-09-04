import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from enum import Enum
import math
import time
from payload.track import compute_gimbal_command
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

class GimbalState(Enum):
    MAPPING = 0
    MANUAL_CONTROL = 1
    LOCK_ON = 2
    SPIN = 3

class PayloadNode(Node):
    def __init__(self):
        super().__init__("payload_node")
        # self.get_logger().info("Payload node started")

        # Declare parameters
        self.declare_parameter("payload_id", "default_payload")
        self.declare_parameter("payload_type", "default_type")

        # Get parameters
        self.payload_id = self.get_parameter("payload_id").get_parameter_value().string_value
        self.payload_type = self.get_parameter("payload_type").get_parameter_value().string_value

        # State machine variables
        self.current_state = GimbalState.MANUAL_CONTROL
        self.current_angle = Vector3()
        self.target_angle = Vector3()
        self.detection_center = Vector3()  # Store detection center coordinates
        self.detection_found = False
        self.spin_increment = 5.0  # Degrees
        self.spin_current_angle = 0.0
        
        # Camera intrinsics configuration
        self.camera_mode = "EO"  # Default camera mode
        self.camera_intrinsics = {
            "EO": {"fx": 475.0, "fy": 505, "cx": 320.0, "cy": 256.0},
            # "RGB_SD": {"width": 640.0, "height": 480.0, "fx": 475.0, "fy": 355.5, "cx": 320.5, "cy": 256.0},
            "IR": {"fx": 2267.0, "fy": 1593.0, "cx": 640.0, "cy": 360.0}
        }
        
        # Add timestamp for logging control
        self.last_state_log_time = 0.0
        
        # Add tracking for last logged state to prevent repeated logging
        self.last_logged_state = None

        # Publishers and subscribers
        # Create a QoS profile for gimbal_angle_pub and gimbal_angle_sub
        gimbal_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # 改为TRANSIENT_LOCAL以兼容rosbag2
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # use gimbal_qos for gimbal_angle_pub and gimbal_angle_sub
        self.gimbal_angle_pub = self.create_publisher(Vector3, '/gimbal_angles', qos_profile=gimbal_qos)
        self.gimbal_angle_sub = self.create_subscription(
            Vector3, 
            '/current_gimbal_angles', 
            self.gimbal_angle_callback, 
            qos_profile=gimbal_qos)
        self.command_sub = self.create_subscription(
            String,
            '/gimbal_command',
            self.command_callback,
            10)
        self.detection_sub = self.create_subscription(
            Vector3,
            '/detection_center',
            self.detection_callback,
            10)
        self.camera_mode_sub = self.create_subscription(
            String,
            '/camera_mode',
            self.camera_mode_callback,
            10)
        # Timers for different states
        self.state_timer = self.create_timer(0.2, self.state_machine_callback)

        # # Camera frame dimensions (example values, adjust based on your camera)
        # self.frame_width = 640  
        # self.frame_height = 360
        
        # Debug counter for detection callback
        self.detection_callback_count = 0
        # self.get_logger().info("Detection subscriber initialized on topic: /detection_box")

    def state_machine_callback(self):
        """Main state machine logic"""
        current_time = time.time()
        
        # Debug print for state machine
        # self.get_logger().info(f"[STATE_MACHINE] Current state: {self.current_state.name}")
        
        if self.current_state == GimbalState.MAPPING:
            self.mapping_mode()
        elif self.current_state == GimbalState.MANUAL_CONTROL:
            self.manual_control_mode()
        elif self.current_state == GimbalState.LOCK_ON:
            # The actual tracking logic is handled in detection_callback
            # which updates self.target_angle based on detections.
            # If no detection, the gimbal should hold its last position
            # (self.target_angle remains unchanged until a new detection).
            # self.get_logger().info(f"[LOCK_ON] In state machine. Detection found: {self.detection_found}")
            pass # No fixed angle setting needed here.
        elif self.current_state == GimbalState.SPIN:
            self.spin_mode()
        
        # Only log state if it has changed
        if self.last_logged_state != self.current_state:
            # self.get_logger().debug(f"Current state: {self.current_state.name}")
            self.last_logged_state = self.current_state
            
        if self.current_state == GimbalState.LOCK_ON or self.current_state == GimbalState.SPIN:
            self.publish_gimbal_angles()

    def mapping_mode(self):
        """Define mapping mode behavior"""
        # Set gimbal to point straight down
        self.target_angle.x = -90.0  # Pitch down 90 degrees
        self.target_angle.y = 0.0
        self.target_angle.z = 0.0
        # Let the state machine publish the angle

    def manual_control_mode(self):
        """Pass-through mode for manual gimbal control"""
        # In this mode, the gimbal simply listens to /gimbal_command
        # and the target angles are set by the gimbal_angle_callback directly
        pass

    def spin_mode(self):
        """Spin the gimbal continuously, panning yaw from -120 to 120 degrees with pitch at -70"""
        # Define the range for yaw and the fixed pitch
        min_yaw = -120.0
        max_yaw = 120.0
        fixed_pitch = -70.0

        # Update the yaw angle
        self.spin_current_angle += self.spin_increment

        # Reverse direction if we reach the min or max yaw
        if self.spin_current_angle > max_yaw:
            self.spin_increment *= -1
            self.spin_current_angle = max_yaw
        elif self.spin_current_angle < min_yaw:
            self.spin_increment *= -1
            self.spin_current_angle = min_yaw

        # Set the target angles
        self.target_angle.x = fixed_pitch  # Pitch
        self.target_angle.z = self.spin_current_angle  # Yaw
        # Let the state machine publish the angle

    def clip_angles(self, angle):
        """Clips the gimbal angles to their valid ranges
        
        Args:
            angle: geometry_msgs.msg.Vector3 containing pitch (x), roll (y), and yaw (z)
        Returns:
            geometry_msgs.msg.Vector3 with clipped values
        """
        # Clip pitch 
        angle.x = max(-120.0, min(120.0, angle.x))
        
        # Fix roll at 0 degrees - no movement allowed
        angle.y = 0.0
        
        # Clip yaw between -170 and 170 degrees
        angle.z = max(-170.0, min(170.0, angle.z))
        
        return angle

    def publish_gimbal_angles(self):
        """Publishes the target gimbal angles"""
        # Clip angles before publishing
        clipped_target_angle = self.clip_angles(self.target_angle) # Create a copy to avoid modifying self.target_angle directly here
        
        # Logic to prevent crossing 0 yaw directly
        # if self.current_angle.z * clipped_target_angle.z < 0:
        #     temp_angle_z = clipped_target_angle.z
        #     clipped_target_angle.z = 0.0
        #     self.gimbal_angle_pub.publish(clipped_target_angle)
        #     # self.get_logger().debug(f"Publishing gimbal angles to 0 yaw first: pitch={clipped_target_angle.x}, roll={clipped_target_angle.y}, yaw={clipped_target_angle.z}")
        #     # time.sleep(0.1) # Consider if a sleep is truly necessary or if sequence is enough
        #     clipped_target_angle.z = temp_angle_z
        #     self.gimbal_angle_pub.publish(clipped_target_angle)
        #     # self.get_logger().debug(f"Publishing gimbal angles: pitch={clipped_target_angle.x}, roll={clipped_target_angle.y}, yaw={clipped_target_angle.z}")
        # else:
        self.gimbal_angle_pub.publish(clipped_target_angle)
        # self.get_logger().debug(f"Publishing gimbal angles: pitch={clipped_target_angle.x}, roll={clipped_target_angle.y}, yaw={clipped_target_angle.z}")


    def gimbal_angle_callback(self, msg):
        """Updates current gimbal angle from subscribed topic"""
        self.current_angle = msg
        # In MANUAL_CONTROL mode, directly update target angle from commanded angle
        # In other modes, self.target_angle is controlled by the state machine or detection logic
        if self.current_state == GimbalState.MANUAL_CONTROL:
             self.target_angle.x = msg.x
             self.target_angle.y = msg.y
             self.target_angle.z = msg.z
             # No need to publish here, MANUAL mode doesn't actively publish based on timer
        # self.get_logger().debug(f"Current gimbal angle: pitch={msg.x}, roll={msg.y}, yaw={msg.z}")

    def command_callback(self, msg):
        """Processes commands from ground station"""
        command = msg.data.strip().upper()
        # self.get_logger().info(f"Received command: {command}")

        if command == "MAPPING":
            self.transition_to_state(GimbalState.MAPPING)
        elif command == "MANUAL":
            self.transition_to_state(GimbalState.MANUAL_CONTROL)
        elif command == "LOCK_ON":
            self.transition_to_state(GimbalState.LOCK_ON)
            # self.get_logger().info("[COMMAND] LOCK_ON mode activated")
            # No need to set fixed angle here, tracking logic handles it
        elif command == "SPIN":
            self.transition_to_state(GimbalState.SPIN)
        else:
            self.get_logger().warn(f"Unknown command: {command}")

    def camera_mode_callback(self, msg):
        """Updates camera mode from subscribed topic"""
        mode = msg.data.strip().upper()
        if mode in self.camera_intrinsics:
            if mode != self.camera_mode:  # Only log if mode has changed
                self.camera_mode = mode
                # self.get_logger().info(f"Camera mode set to: {self.camera_mode}")
        else:
            self.get_logger().warn(f"Unknown camera mode: {mode}. Using current mode: {self.camera_mode}")

    def detection_callback(self, msg):
        """Process detection center coordinates"""
        self.detection_callback_count += 1
        
        # Update detection center
        self.detection_center = msg
        
        # Check if detection is valid (non-zero coordinates indicate detection)
        self.detection_found = (msg.x != 0.0 or msg.y != 0.0)
        
        if self.detection_found:
            # Keep this as it's important tracking info
            self.get_logger().info(f"[DETECTION] Detected object at center: x={msg.x}, y={msg.y}")
            
            # Track only on LOCK_ON state
            if self.current_state == GimbalState.LOCK_ON:
                # self.get_logger().info(f"[LOCK_ON] Processing detection in LOCK_ON mode")
                
                # Use detection center coordinates directly
                center_x = msg.x
                center_y = msg.y
                
                # Keep this for tracking info
                # self.get_logger().info(f"[LOCK_ON] Detection center: x={center_x}, y={center_y}")
                
                # Get current camera intrinsics based on camera mode
                intrinsics = self.camera_intrinsics[self.camera_mode]
                cx = intrinsics["cx"]
                cy = intrinsics["cy"]
                fx = intrinsics["fx"]
                fy = intrinsics["fy"]
                
                try:
                    new_pitch, new_yaw = compute_gimbal_command(
                        center_x, center_y, 
                        cx, cy, 
                        fx, fy, 
                        self.current_angle.x, self.current_angle.z
                    )
                    
                    # Keep track of the new angles as they indicate tracking success
                    self.get_logger().info(f"[LOCK_ON] compute_gimbal_command output: new_pitch={new_pitch}, new_yaw={new_yaw}")

                    # Update the target angles based on the computed command
                    self.target_angle.z = new_yaw
                    self.target_angle.x = new_pitch # pitch
                    self.target_angle.y = 0.0 # Keep roll fixed

                    # Keep this as it shows tracking state
                    # self.get_logger().info(f"[LOCK_ON] New target angles: pitch={self.target_angle.x}, yaw={self.target_angle.z}")
                    
                    # Publish new gimbal angles immediately after computation for responsiveness
                    # The state machine will also publish periodically, but immediate publish helps tracking
                    self.publish_gimbal_angles() 
                except Exception as e:
                    # Keep errors as they indicate tracking failures
                    self.get_logger().error(f"[LOCK_ON] Error in compute_gimbal_command: {str(e)}")
        # else:
            # If no detection found while in LOCK_ON mode, gimbal holds its last target position
            # self.get_logger().info("[DETECTION] No objects detected")
            # pass # No need to do anything, self.target_angle retains its value

    def transition_to_state(self, new_state):
        """Handle state transitions"""
        if new_state == self.current_state:
            # self.get_logger().info(f"[STATE] Already in {new_state.name} state, no transition needed")
            return
        
        # self.get_logger().info(f"[STATE] Transitioning from {self.current_state.name} to {new_state.name}")
        
        # Exit actions for current state
        if self.current_state == GimbalState.SPIN:
            # Reset spin variables  
            self.spin_current_angle = 0.0
        
        # Entry actions for new state
        if new_state == GimbalState.MAPPING:
            # Set initial target for mapping mode
            self.target_angle.x = -90.0  # Pitch down 90 degrees
            self.target_angle.y = 0.0 
            self.target_angle.z = 0.0
            self.publish_gimbal_angles() # Publish initial mapping angle
        elif new_state == GimbalState.LOCK_ON:
            # When entering LOCK_ON, set initial pitch to -50 degrees.
            # Use the current yaw as the initial target yaw.
            # The gimbal will move to this position and hold it
            # or start tracking immediately if a detection occurs.
            self.target_angle.x = -50.0  # Set pitch to -50 degrees
            self.target_angle.y = 0.0    # Roll remains 0
            self.target_angle.z = self.current_angle.z # Maintain current yaw initially
            self.get_logger().info(f"[STATE] Entering LOCK_ON mode. Setting initial pitch to -50, yaw to {self.target_angle.z}. Waiting for detections.")
            self.publish_gimbal_angles() # Publish the initial target angle
            # Ensure detection status is logged
            # self.get_logger().info(f"Initial detection_found status: {self.detection_found}")


        # Reset last_logged_state to force a new log message
        self.last_logged_state = None
        self.current_state = new_state


def main():
    rclpy.init()
    payload_node = PayloadNode()
    try:
        rclpy.spin(payload_node)
    except KeyboardInterrupt:
        pass
    finally:
        payload_node.destroy_node()
        rclpy.shutdown()
        # payload_node.get_logger().info("Payload node shutting down")

if __name__ == "__main__":
    main()
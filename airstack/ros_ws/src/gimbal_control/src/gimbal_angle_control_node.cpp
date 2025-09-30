#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "payloadSdkInterface.h"
#include <csignal>
#include <vector>

// Fallback definitions if specific platform SDK headers did not define them
#ifndef ZOOM_OUT
#define ZOOM_OUT -1
#endif
#ifndef ZOOM_STOP
#define ZOOM_STOP 0
#endif
#ifndef ZOOM_IN
#define ZOOM_IN 1
#endif
#ifndef ZOOM_TYPE_CONTINUOUS
#define ZOOM_TYPE_CONTINUOUS 1
#endif

T_ConnInfo s_conn = {
	CONTROL_UDP,
	{udp_ip_target, udp_port_target}
};

class GimbalAngleControlNode : public rclcpp::Node {
public:
	GimbalAngleControlNode() : Node("gimbal_teleop_control_node") {
		RCLCPP_INFO(this->get_logger(), "Gimbal teleop control node started");
		// Setup subscribers
		// Subscribe to Joy messages from Foxglove panel
		joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
			"/gimbal/teleop", 10,
			std::bind(&GimbalAngleControlNode::joy_callback, this, std::placeholders::_1));
		// Keep Vector3 subscriber for backwards compatibility
		teleop_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>(
			"/gimbal/teleop_vector", 10,
			std::bind(&GimbalAngleControlNode::teleop_callback, this, std::placeholders::_1));
		angle_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>(
			"gimbal_angles", 10,
			std::bind(&GimbalAngleControlNode::angle_callback, this, std::placeholders::_1));

		// Setup publisher for current angles
		auto qos = rclcpp::QoS(10).transient_local();
		current_angles_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("current_gimbal_angles", qos);

		// Setup timer to read gimbal angles every 50ms
		timer_ = this->create_wall_timer(
			std::chrono::milliseconds(50),
			std::bind(&GimbalAngleControlNode::read_gimbal_angles, this));

		// Initialize SDK
		try {
			my_payload_ = new PayloadSdkInterface(s_conn);
			RCLCPP_INFO(this->get_logger(), "Initializing PayloadSDK connection...");
			my_payload_->sdkInitConnection();
			RCLCPP_INFO(this->get_logger(), "Checking payload connection...");
			my_payload_->checkPayloadConnection();
			RCLCPP_INFO(this->get_logger(), "PayloadSDK initialized successfully");
		} catch (...) {
			RCLCPP_ERROR(this->get_logger(), "Failed to initialize PayloadSDK");
			if (my_payload_ != nullptr) {
				delete my_payload_;
				my_payload_ = nullptr;
			}
		}

		// Declare configurable parameters (can be overridden via ROS2 CLI/launch)
		this->declare_parameter<int>("zoom_axis", 2); // axis index for zoom control
		this->declare_parameter<double>("zoom_deadband", 0.05);
		this->declare_parameter<int>("eo_ir_toggle_button", 3); // button index to toggle EO/IR
		// Defaults assume MIO is EO and VIO is IR; can be overridden
		this->declare_parameter<int>("eo_state", static_cast<int>(CABLE_STATE_MIO));
		this->declare_parameter<int>("ir_state", static_cast<int>(CABLE_STATE_VIO));

		zoom_axis_ = this->get_parameter("zoom_axis").as_int();
		zoom_deadband_ = this->get_parameter("zoom_deadband").as_double();
		eo_ir_toggle_button_ = this->get_parameter("eo_ir_toggle_button").as_int();
		eo_state_ = this->get_parameter("eo_state").as_int();
		ir_state_ = this->get_parameter("ir_state").as_int();

		RCLCPP_INFO(this->get_logger(), "Node ready to receive commands");
		RCLCPP_INFO(this->get_logger(), "Primary topic: /gimbal/teleop (Joy messages)");
		RCLCPP_INFO(this->get_logger(), "Backup topic: /gimbal/teleop_vector (Vector3)");
		RCLCPP_INFO(this->get_logger(), "Angle topic: gimbal_angles");
		RCLCPP_INFO(this->get_logger(), "Zoom axis=%d deadband=%.2f", zoom_axis_, zoom_deadband_);
		RCLCPP_INFO(this->get_logger(), "EO/IR toggle button index=%d (EO state=%d IR state=%d)", eo_ir_toggle_button_, eo_state_, ir_state_);
	}

	~GimbalAngleControlNode() {
		if (my_payload_ != nullptr) {
			delete my_payload_;
		}
	}

private:
	void angle_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
		RCLCPP_DEBUG(this->get_logger(), "Received local angle command: [%.2f, %.2f, %.2f]", msg->x, msg->y, msg->z);
		set_gimbal_angle(msg->x, msg->y, msg->z);
	}

	void teleop_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
		RCLCPP_DEBUG(this->get_logger(), "Received teleop command from operator: [%.2f, %.2f, %.2f]", msg->x, msg->y, msg->z);
		set_gimbal_angle(msg->x, msg->y, msg->z, true); // true indicates rate mode
	}

	void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
		// Convert Joy message to gimbal rate commands
		// axes[0] = panX (-1..1), axes[1] = panY (-1..1), axes[2] = zoom (-1/0/1)
		// For gimbal: x=pitch, y=yaw, z=roll
		// Map panY to pitch (up/down), panX to yaw (left/right)
		
		if (msg->axes.size() < 2) {
			RCLCPP_WARN(this->get_logger(), "Joy message has insufficient axes data");
			return;
		}
		
		float pan_x = msg->axes[0];  // left/right movement (-1 to 1)
		float pan_y = msg->axes[1];  // up/down movement (-1 to 1)
		float zoom = (zoom_axis_ >= 0 && static_cast<size_t>(zoom_axis_) < msg->axes.size()) ? msg->axes[zoom_axis_] : 0.0f; // zoom control (continuous)
		
		// Check if any movement is being commanded (non-zero axes)
		bool is_moving = (std::abs(pan_x) > 0.01f || std::abs(pan_y) > 0.01f);
		
		if (is_moving) {
			// Convert normalized joystick values to gimbal rate commands
			// Scale to reasonable degrees per second (adjust as needed)
			float max_rate = 30.0f; // degrees per second
			float pitch_rate = pan_y * max_rate; // positive pan_y = up = positive pitch
			float yaw_rate = pan_x * max_rate;   // positive pan_x = right = positive yaw
			float roll_rate = 0.0f;              // not using roll for now
			
			RCLCPP_DEBUG(this->get_logger(), "Joy movement command: panX=%.2f, panY=%.2f -> pitch_rate=%.2f°/s, yaw_rate=%.2f°/s", 
				pan_x, pan_y, pitch_rate, yaw_rate);
			
			// Send rate commands to gimbal
			set_gimbal_angle(pitch_rate, yaw_rate, roll_rate, true); // true = rate mode
		} else {
			// No movement commanded - stop gimbal movement
			RCLCPP_DEBUG(this->get_logger(), "Joy stop command - no movement");
			set_gimbal_angle(0.0f, 0.0f, 0.0f, true); // stop all movement
		}
		
		// Handle zoom commands (continuous). Correct usage: setCameraZoom(ZOOM_TYPE_CONTINUOUS, ZOOM_[IN/OUT/STOP])
		if (my_payload_ != nullptr) {
			bool above_deadband = std::abs(zoom) > zoom_deadband_;
			int desired_zoom_cmd = ZOOM_STOP;
			if (above_deadband) {
				desired_zoom_cmd = (zoom > 0) ? ZOOM_IN : ZOOM_OUT;
			}
			if (desired_zoom_cmd != last_zoom_cmd_) {
				try {
					my_payload_->setCameraZoom(ZOOM_TYPE_CONTINUOUS, desired_zoom_cmd);
					if (desired_zoom_cmd == ZOOM_IN) {
						RCLCPP_DEBUG(this->get_logger(), "Zoom IN start");
					} else if (desired_zoom_cmd == ZOOM_OUT) {
						RCLCPP_DEBUG(this->get_logger(), "Zoom OUT start");
					} else {
						RCLCPP_DEBUG(this->get_logger(), "Zoom STOP");
					}
					last_zoom_cmd_ = desired_zoom_cmd;
				} catch (const std::exception& e) {
					RCLCPP_ERROR(this->get_logger(), "Failed to control zoom: %s", e.what());
				}
			}
		}
		
		// Handle button presses for special actions
		if (msg->buttons.size() >= 3) {
			if (msg->buttons[2] > 0) { // Recenter button (index 2)
				RCLCPP_INFO(this->get_logger(), "Recenter command received");
				try {
					if (my_payload_ != nullptr) {
						// Reset gimbal to center position using PayloadSDK
						my_payload_->setGimbalResetMode(Gimbal_Protocol::GIMBAL_RESET_MODE_PITCH_AND_YAW);
					}
					// Also stop all movement
					set_gimbal_angle(0.0f, 0.0f, 0.0f, true);
				} catch (const std::exception& e) {
					RCLCPP_ERROR(this->get_logger(), "Failed to recenter gimbal: %s", e.what());
				}
			}
			
			if (msg->buttons[0] > 0) { // Trigger button (index 0)
				RCLCPP_INFO(this->get_logger(), "Trigger/Capture button pressed");
				try {
					if (my_payload_ != nullptr) {
						// Capture image using PayloadSDK
						my_payload_->setPayloadCameraCaptureImage();
					}
				} catch (const std::exception& e) {
					RCLCPP_ERROR(this->get_logger(), "Failed to capture image: %s", e.what());
				}
			}
			
			if (msg->buttons[1] > 0) { // Mode switch button (index 1)
				RCLCPP_INFO(this->get_logger(), "Mode switch button pressed");
				try {
					if (my_payload_ != nullptr) {
						// Toggle between LOCK and FOLLOW modes
						static bool is_lock_mode = false;
						if (is_lock_mode) {
							my_payload_->setGimbalMode(Gimbal_Protocol::GIMBAL_FOLLOW_MODE);
							RCLCPP_INFO(this->get_logger(), "Switched to FOLLOW mode");
						} else {
							my_payload_->setGimbalMode(Gimbal_Protocol::GIMBAL_LOCK_MODE);
							RCLCPP_INFO(this->get_logger(), "Switched to LOCK mode");
						}
						is_lock_mode = !is_lock_mode;
					}
				} catch (const std::exception& e) {
					RCLCPP_ERROR(this->get_logger(), "Failed to switch gimbal mode: %s", e.what());
				}
			}

			// EO/IR toggle (rising edge detection)
			if (eo_ir_toggle_button_ >= 0 && static_cast<size_t>(eo_ir_toggle_button_) < msg->buttons.size()) {
				bool pressed = msg->buttons[eo_ir_toggle_button_] > 0;
				if (pressed && !last_eo_ir_pressed_) {
					try {
						if (my_payload_ != nullptr) {
							int target_state = using_eo_ ? ir_state_ : eo_state_;
							my_payload_->setGimbalICSwitch(static_cast<cable_state_e>(target_state));
							using_eo_ = !using_eo_;
							RCLCPP_INFO(this->get_logger(), "Switched to %s camera (state=%d)", using_eo_ ? "EO" : "IR", target_state);
						}
					} catch (const std::exception& e) {
						RCLCPP_ERROR(this->get_logger(), "Failed to toggle EO/IR: %s", e.what());
					}
				}
				last_eo_ir_pressed_ = pressed;
			}
		}
	}

	void set_gimbal_angle(float pitch, float yaw, float roll, bool is_rate_mode = false) {
		if (my_payload_ == nullptr) {
			RCLCPP_ERROR(this->get_logger(), "Gimbal not connected (my_payload_ is null)");
			return;
		}
		try {
			RCLCPP_DEBUG(this->get_logger(), "Setting gimbal control...");
			if (is_rate_mode) {
				// For teleop rate commands - use INPUT_SPEED for continuous movement
				my_payload_->setGimbalSpeed(pitch, roll, yaw, Gimbal_Protocol::INPUT_SPEED);
			} else {
				// For absolute angle commands
				my_payload_->setGimbalSpeed(pitch, roll, yaw, Gimbal_Protocol::INPUT_ANGLE);
			}
			usleep(10000); // Short sleep for responsiveness
		} catch (const std::exception& e) {
			RCLCPP_ERROR(this->get_logger(), "Failed to set gimbal control: %s", e.what());
		} catch (...) {
			RCLCPP_ERROR(this->get_logger(), "Unknown error setting gimbal control");
		}
	}

	void read_gimbal_angles() {
		if (my_payload_ == nullptr) {
			return;
		}
		try {
			mavlink_message_t msg_in;
			uint8_t msg_cnt = my_payload_->getNewMewssage(msg_in);
			if (msg_cnt && msg_in.msgid == MAVLINK_MSG_ID_MOUNT_ORIENTATION) {
				mavlink_mount_orientation_t mount_orientation;
				mavlink_msg_mount_orientation_decode(&msg_in, &mount_orientation);
				auto msg_out = geometry_msgs::msg::Vector3Stamped();
				msg_out.header.stamp = this->get_clock()->now();
				msg_out.vector.x = mount_orientation.roll;
				msg_out.vector.y = mount_orientation.pitch;
				msg_out.vector.z = mount_orientation.yaw;
				current_angles_publisher_->publish(msg_out);
			}
		} catch (const std::exception& e) {
			RCLCPP_ERROR(this->get_logger(), "Error reading gimbal angles: %s", e.what());
		} catch (...) {
			RCLCPP_ERROR(this->get_logger(), "Unknown error reading gimbal angles");
		}
	}

	rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angle_subscriber_;
	rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr teleop_subscriber_;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr current_angles_publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
	PayloadSdkInterface* my_payload_{nullptr};

	// Zoom state tracking
	int last_zoom_cmd_ {ZOOM_STOP};
	int zoom_axis_ {2};
	double zoom_deadband_ {0.05};

	// EO/IR switching
	int eo_ir_toggle_button_ {3};
	int eo_state_ {static_cast<int>(CABLE_STATE_MIO)}; // assumed EO
	int ir_state_ {static_cast<int>(CABLE_STATE_VIO)}; // assumed IR
	bool using_eo_ {true};
	bool last_eo_ir_pressed_ {false};
};

void signal_handler(int signum) {
	(void)signum;
	rclcpp::shutdown();
}

int main(int argc, char** argv) {
	signal(SIGINT, signal_handler);
	rclcpp::init(argc, argv);
	auto node = std::make_shared<GimbalAngleControlNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
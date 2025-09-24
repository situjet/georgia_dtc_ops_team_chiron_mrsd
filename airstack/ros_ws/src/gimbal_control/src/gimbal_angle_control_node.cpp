#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "payloadSdkInterface.h"
#include <csignal>

// PayloadSDK constants - ensure these are defined
#ifndef ZOOM_TYPE_CONTINUOUS
#define ZOOM_TYPE_CONTINUOUS 1
#endif
#ifndef ZOOM_TYPE_STEP
#define ZOOM_TYPE_STEP 0
#endif
#ifndef ZOOM_IN
#define ZOOM_IN 1
#endif
#ifndef ZOOM_OUT
#define ZOOM_OUT -1
#endif
#ifndef ZOOM_STOP
#define ZOOM_STOP 0
#endif
#ifndef PAYLOAD_CAMERA_VIEW_EO
#define PAYLOAD_CAMERA_VIEW_EO 1
#endif
#ifndef PAYLOAD_CAMERA_VIEW_IR
#define PAYLOAD_CAMERA_VIEW_IR 2
#endif
#ifndef PAYLOAD_CAMERA_RECORD_BOTH
#define PAYLOAD_CAMERA_RECORD_BOTH 0
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

		RCLCPP_INFO(this->get_logger(), "Node ready to receive commands");
		RCLCPP_INFO(this->get_logger(), "Primary topic: /gimbal/teleop (Joy messages)");
		RCLCPP_INFO(this->get_logger(), "Backup topic: /gimbal/teleop_vector (Vector3)");
		RCLCPP_INFO(this->get_logger(), "Angle topic: gimbal_angles");
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
		float zoom = (msg->axes.size() > 2) ? msg->axes[2] : 0.0f; // zoom control
		
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
		
		// Handle zoom commands
		if (std::abs(zoom) > 0.01f && my_payload_ != nullptr) {
			try {
				if (zoom > 0) {
					RCLCPP_DEBUG(this->get_logger(), "Zoom in command: %.2f", zoom);
					my_payload_->setCameraZoom(ZOOM_TYPE_CONTINUOUS, ZOOM_IN); // Use continuous zoom in
				} else {
					RCLCPP_DEBUG(this->get_logger(), "Zoom out command: %.2f", zoom);
					my_payload_->setCameraZoom(ZOOM_TYPE_CONTINUOUS, ZOOM_OUT); // Use continuous zoom out
				}
			} catch (const std::exception& e) {
				RCLCPP_ERROR(this->get_logger(), "Failed to control zoom: %s", e.what());
			}
		} else if (my_payload_ != nullptr) {
			// Stop zoom when no zoom command is active
			try {
				my_payload_->setCameraZoom(ZOOM_TYPE_CONTINUOUS, ZOOM_STOP);
			} catch (const std::exception& e) {
				RCLCPP_DEBUG(this->get_logger(), "Failed to stop zoom: %s", e.what());
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
			
			if (msg->buttons[0] > 0) { // Trigger button (index 0) - Record both EO and IR for 10 seconds
				RCLCPP_INFO(this->get_logger(), "Record button pressed - starting 10s EO+IR recording");
				try {
					if (my_payload_ != nullptr) {
						// Set recording source to both EO and IR
						my_payload_->setPayloadCameraParam("C_V_REC", PAYLOAD_CAMERA_RECORD_BOTH, PARAM_TYPE_UINT32);
						usleep(100000); // Wait 100ms for setting to take effect
						
						// Start recording
						my_payload_->setPayloadCameraRecordVideoStart();
						
						// Schedule a stop after 10 seconds
						recording_timer_ = this->create_wall_timer(
							std::chrono::seconds(10),
							[this]() {
								try {
									my_payload_->setPayloadCameraRecordVideoStop();
									RCLCPP_INFO(this->get_logger(), "10-second recording completed");
									recording_timer_.reset(); // Clear the timer
								} catch (const std::exception& e) {
									RCLCPP_ERROR(this->get_logger(), "Failed to stop recording: %s", e.what());
								}
							}
						);
					}
				} catch (const std::exception& e) {
					RCLCPP_ERROR(this->get_logger(), "Failed to start recording: %s", e.what());
				}
			}
			
			if (msg->buttons[1] > 0) { // Mode switch button (index 1) - Toggle EO/IR camera view
				RCLCPP_INFO(this->get_logger(), "Camera mode switch button pressed");
				try {
					if (my_payload_ != nullptr) {
						// Toggle between EO and IR camera views
						static bool is_eo_mode = true;
						if (is_eo_mode) {
							my_payload_->setPayloadCameraParam("C_SOURCE", PAYLOAD_CAMERA_VIEW_IR, PARAM_TYPE_UINT32);
							RCLCPP_INFO(this->get_logger(), "Switched to IR camera view");
						} else {
							my_payload_->setPayloadCameraParam("C_SOURCE", PAYLOAD_CAMERA_VIEW_EO, PARAM_TYPE_UINT32);
							RCLCPP_INFO(this->get_logger(), "Switched to EO camera view");
						}
						is_eo_mode = !is_eo_mode;
					}
				} catch (const std::exception& e) {
					RCLCPP_ERROR(this->get_logger(), "Failed to switch camera view: %s", e.what());
				}
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
	rclcpp::TimerBase::SharedPtr recording_timer_;
	PayloadSdkInterface* my_payload_{nullptr};
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
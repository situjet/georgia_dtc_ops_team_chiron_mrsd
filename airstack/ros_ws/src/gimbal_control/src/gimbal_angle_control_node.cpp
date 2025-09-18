// Gimbal angle control node for ROS2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include <csignal>

// Use the same header as the working examples
#include "payloadSdkInterface.h"

// [31mDefine connection parameters[0m
T_ConnInfo s_conn = {
	CONTROL_UDP,
	udp_ip_target,
	udp_port_target
};

class GimbalAngleControlNode : public rclcpp::Node
{
public:
	GimbalAngleControlNode() : Node("gimbal_angle_control_node")
	{
		RCLCPP_INFO(this->get_logger(), "Gimbal angle control node started");
        
		// Create subscriber for angle commands (local)
		angle_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>(
			"gimbal_angles", 10,
			std::bind(&GimbalAngleControlNode::angle_callback, this, std::placeholders::_1));

		// Create subscriber for teleop commands from operator (domain 70)
		teleop_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>(
			"/gimbal/teleop", 10,
			std::bind(&GimbalAngleControlNode::teleop_callback, this, std::placeholders::_1));

		// Initialize SDK
		try {
			my_payload_ = new PayloadSdkInterface(s_conn);
			RCLCPP_INFO(this->get_logger(), "1");
			my_payload_->sdkInitConnection();
			RCLCPP_INFO(this->get_logger(), "2");
			my_payload_->checkPayloadConnection();
			RCLCPP_INFO(this->get_logger(), "3");
            
		// Set gimbal to FOLLOW mode (not LOCK mode)
		my_payload_->setPayloadCameraParam((char*)"GB_MODE", 
			2, 3);  // 2 = FOLLOW mode, 3 = PARAM_TYPE_UINT32
		usleep(1000000);  // Wait for mode switch			RCLCPP_INFO(this->get_logger(), "SDK initialized successfully");
		} catch (...) {
			RCLCPP_ERROR(this->get_logger(), "Failed to initialize SDK");
			if (my_payload_ != nullptr) {
				delete my_payload_;
				my_payload_ = nullptr;
			}
		}

		RCLCPP_INFO(this->get_logger(), "Node is ready to receive angle commands");
		RCLCPP_INFO(this->get_logger(), "Using topics: /gimbal_angles and /gimbal/teleop");

		// Create publisher for current angles
		auto qos = rclcpp::QoS(10).transient_local();
		current_angles_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("current_gimbal_angles", qos);
        
		// Create timer, read gimbal angles every 50ms (20Hz)
		timer_ = this->create_wall_timer(
			std::chrono::milliseconds(50),
			std::bind(&GimbalAngleControlNode::read_gimbal_angles, this));
	}

	~GimbalAngleControlNode()
	{
		if (my_payload_ != nullptr) {
			delete my_payload_;
		}
	}

	void read_gimbal_angles()
	{
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

private:
	void angle_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
	{
		RCLCPP_DEBUG(this->get_logger(), "In angle callback");
		set_gimbal_angle(msg->x, msg->y, msg->z);
	}

	void teleop_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
	{
		RCLCPP_DEBUG(this->get_logger(), "In teleop callback from operator");
		// Convert teleop commands (rate commands) to gimbal control
		// Teleop typically sends rate commands, so we use INPUT_ANGULAR_RATE mode
		set_gimbal_angle(msg->x, msg->y, msg->z, true); // true indicates rate mode
	}

	void set_gimbal_angle(float pitch, float yaw, float roll, bool is_rate_mode = false)
	{
		if (my_payload_ == nullptr) {
			RCLCPP_ERROR(this->get_logger(), "Gimbal not connected (my_payload_ is null)");
			return;
		}

		try {
			RCLCPP_DEBUG(this->get_logger(), "Trying to set gimbal angle...");
			if (is_rate_mode) {
				// For teleop rate commands
				my_payload_->setGimbalSpeed(pitch, roll, yaw, Gimbal_Protocol::INPUT_ANGULAR_RATE);
			} else {
				// For absolute angle commands
				my_payload_->setGimbalSpeed(pitch, roll, yaw, Gimbal_Protocol::INPUT_ANGLE);
			}
			usleep(50000); // Reduce sleep time for better responsiveness
		} catch (const std::exception& e) {
			RCLCPP_ERROR(this->get_logger(), "Failed to set gimbal angle: %s", e.what());
		} catch (...) {
			RCLCPP_ERROR(this->get_logger(), "Unknown error setting gimbal angle");
		}
	}

	rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angle_subscriber_;
	rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr teleop_subscriber_;
	PayloadSdkInterface* my_payload_{nullptr};
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr current_angles_publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
};

void signal_handler(int signum) {
	(void)signum;
	rclcpp::shutdown();
}

int main(int argc, char** argv)
{
	signal(SIGINT, signal_handler);
	rclcpp::init(argc, argv);
	auto node = std::make_shared<GimbalAngleControlNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

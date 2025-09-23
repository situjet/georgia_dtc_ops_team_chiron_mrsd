#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "payloadSdkInterface.h"

T_ConnInfo s_conn = {
	CONTROL_UDP,
	{udp_ip_target, udp_port_target}
};

class GimbalStatusNode : public rclcpp::Node {
public:
	GimbalStatusNode() : Node("gimbal_status_node") {
		attitude_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/gimbal_attitude", 10);
		initializePayloadSDK();
		timer_ = this->create_wall_timer(
			std::chrono::milliseconds(10),
			std::bind(&GimbalStatusNode::timer_callback, this));
		RCLCPP_INFO(this->get_logger(), "GimbalStatusNode initialized");
	}

	~GimbalStatusNode() {
		if (my_payload_) {
			my_payload_->sdkQuit();
			delete my_payload_;
		}
	}

private:
	void initializePayloadSDK() {
		try {
			my_payload_ = new PayloadSdkInterface(s_conn);
			my_payload_->sdkInitConnection();
			my_payload_->checkPayloadConnection();
			my_payload_->regPayloadStatusChanged(
				std::bind(&GimbalStatusNode::onPayloadStatusChanged, this, 
				std::placeholders::_1, std::placeholders::_2));
			for (uint8_t i = 0; i < PARAM_COUNT; i++) {
				my_payload_->setParamRate(i, 10);
			}
			RCLCPP_INFO(this->get_logger(), "PayloadSDK initialized successfully");
		} catch (const std::exception& e) {
			RCLCPP_ERROR(this->get_logger(), "Failed to initialize PayloadSDK: %s", e.what());
			throw;
		}
	}

	void timer_callback() {
		if (!my_payload_) return;
	}

	void onPayloadStatusChanged(int event, double* param) {
		switch(event) {
			case PAYLOAD_GB_ATTITUDE: {
				auto msg = geometry_msgs::msg::Point();
				msg.x = param[0];
				msg.y = param[1];
				msg.z = param[2];
				attitude_publisher_->publish(msg);
				break;
			}
		}
	}

	PayloadSdkInterface* my_payload_{nullptr};
	rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr attitude_publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<GimbalStatusNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

class GimbalControlNode : public rclcpp::Node {
public:
    GimbalControlNode() : Node("gimbal_control_node") {
        RCLCPP_INFO(this->get_logger(), "Gimbal control node started");
        angle_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "gimbal_angles", 10,
            std::bind(&GimbalControlNode::angle_callback, this, std::placeholders::_1));
        // TODO: Initialize gimbal connection here
    }
private:
    void angle_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        // TODO: Send angle command to gimbal hardware
        RCLCPP_INFO(this->get_logger(), "Received angles: [%.2f, %.2f, %.2f]", msg->x, msg->y, msg->z);
    }
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angle_sub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GimbalControlNode>());
    rclcpp::shutdown();
    return 0;
}

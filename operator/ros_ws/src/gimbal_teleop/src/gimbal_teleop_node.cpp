#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/vector3.hpp>

class GimbalTeleop : public rclcpp::Node
{
public:
    GimbalTeleop() : Node("gimbal_teleop_node")
    {
        RCLCPP_INFO(this->get_logger(), "Gimbal teleop node started");
        
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, 
            std::bind(&GimbalTeleop::joyCallback, this, std::placeholders::_1));
        
        gimbal_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/gimbal/teleop", 10);
        
        RCLCPP_INFO(this->get_logger(), "Publishing gimbal commands to /gimbal/teleop");
        RCLCPP_INFO(this->get_logger(), "Subscribing to joystick on /joy");
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
    {
        geometry_msgs::msg::Vector3 cmd;
        // Example mapping: right stick X (axes[3]) = yaw, right stick Y (axes[4]) = pitch
        // Triggers (axes[2], axes[5]) for zoom (if needed)
        
        // Make sure we have enough axes
        if (joy->axes.size() >= 6) {
            cmd.x = joy->axes[4]; // pitch
            cmd.y = joy->axes[3]; // yaw  
            cmd.z = (joy->axes[5] - joy->axes[2]) * 0.5; // zoom (example)
            
            gimbal_pub_->publish(cmd);
            
            // Log only when there's significant input
            if (abs(cmd.x) > 0.1 || abs(cmd.y) > 0.1 || abs(cmd.z) > 0.1) {
                RCLCPP_DEBUG(this->get_logger(), 
                    "Gimbal command: pitch=%.2f, yaw=%.2f, zoom=%.2f", 
                    cmd.x, cmd.y, cmd.z);
            }
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Joy message has insufficient axes: %zu", joy->axes.size());
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gimbal_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GimbalTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

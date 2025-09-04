#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "payloadSdkInterface.h"

// 定义连接参数
T_ConnInfo s_conn = {
    CONTROL_UDP,
    udp_ip_target,
    udp_port_target
};

class GimbalStatusNode : public rclcpp::Node {
public:
    GimbalStatusNode() : Node("gimbal_status_node") {
        // 使用 Point 类型创建发布者
        attitude_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/gimbal_attitude", 10);

        // 初始化SDK
        initializePayloadSDK();

        // 创建定时器，100Hz
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
            
            // 注册回调函数
            my_payload_->regPayloadStatusChanged(
                std::bind(&GimbalStatusNode::onPayloadStatusChanged, this, 
                std::placeholders::_1, std::placeholders::_2));
            
            // 尝试设置云台姿态数据的更新频率为100Hz (10ms)
            // PAYLOAD_GB_ATTITUDE 是枚举值，可能需要找到对应的参数索引
            for (uint8_t i = 0; i < PARAM_COUNT; i++) {
                my_payload_->setParamRate(i, 10); // 设置所有参数以10ms的间隔更新
            }
            
            RCLCPP_INFO(this->get_logger(), "PayloadSDK initialized successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize PayloadSDK: %s", e.what());
            throw;
        }
    }

    void timer_callback() {
        if (!my_payload_) return;
        // 定时器只用于保持节点活跃，实际数据通过回调函数处理
    }

    void onPayloadStatusChanged(int event, double* param) {
        switch(event) {
            case PAYLOAD_GB_ATTITUDE: {
                auto msg = geometry_msgs::msg::Point();
                
                // 直接使用欧拉角
                msg.x = param[0];  // pitch in degrees
                msg.y = param[1];  // roll in degrees
                msg.z = param[2];  // yaw in degrees

                attitude_publisher_->publish(msg);
                // RCLCPP_INFO(this->get_logger(), "Published attitude - Pitch: %.2f, Roll: %.2f, Yaw: %.2f",
                //     param[0], param[1], param[2]);
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
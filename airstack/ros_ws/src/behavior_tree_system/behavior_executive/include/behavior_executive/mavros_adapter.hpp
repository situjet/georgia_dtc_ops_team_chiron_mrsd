#pragma once

#include "mavros_adapter_interface.hpp"

#include <behavior_tree_msgs/msg/active.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/waypoint_clear.hpp>
#include <mavros_msgs/srv/waypoint_push.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <std_msgs/msg/string.hpp>

/**
 * @brief MAVROS 适配器的具体实现
 * 
 * 此类封装了所有实际的 ROS/MAVROS 服务调用和消息发布操作。
 * 它持有 ROS 客户端和发布者的引用，并实现 IMavrosAdapter 接口。
 */
class MavrosAdapter : public IMavrosAdapter {
public:
    /**
     * @brief 构造函数
     * @param node ROS2 节点指针，用于创建客户端和发布者
     */
    explicit MavrosAdapter(rclcpp::Node* node);

    void set_mode(const std::string& mode, 
                 SetModeCallback callback,
                 const std::string& logger_name) override;

    void arm_vehicle(ArmCallback callback,
                    const std::string& logger_name) override;

    void disarm_vehicle(ArmCallback callback,
                       const std::string& logger_name) override;

    void clear_waypoints(WaypointCallback callback,
                        const std::string& logger_name) override;

    void push_waypoints(const std::vector<sensor_msgs::msg::NavSatFix>& waypoints,
                       double target_altitude,
                       double hold_time,
                       const std::vector<double>& yaws,
                       double acceptance_radius,
                       WaypointCallback callback,
                       const std::string& logger_name,
                       const std::string& current_mode) override;

    void publish_gimbal_command(const std::string& command, int count = 1) override;

    void publish_hold_active(int id) override;

private:
    rclcpp::Node* node_;
    
    // 回调组（必须保持存活以便服务回调能够执行）
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;

    // ROS 客户端
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_disarm_command_client_;
    rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr waypoint_push_client_;
    rclcpp::Client<mavros_msgs::srv::WaypointClear>::SharedPtr waypoint_clear_client_;

    // ROS 发布者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gimbal_command_pub_;
    rclcpp::Publisher<behavior_tree_msgs::msg::Active>::SharedPtr hold_active_pub_;
};


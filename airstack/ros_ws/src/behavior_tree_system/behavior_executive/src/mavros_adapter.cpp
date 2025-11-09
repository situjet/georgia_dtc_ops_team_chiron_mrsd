#include "mavros_adapter.hpp"

#include <chrono>
#include <mavros_msgs/msg/waypoint.hpp>

using namespace std::chrono_literals;

MavrosAdapter::MavrosAdapter(rclcpp::Node* node) : node_(node) {
    // 创建回调组用于服务调用（必须保存为成员变量以保持存活）
    service_callback_group_ =
        node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // 创建服务客户端
    set_mode_client_ = node_->create_client<mavros_msgs::srv::SetMode>(
        "mavros/set_mode", rmw_qos_profile_services_default, service_callback_group_);
    
    arm_disarm_command_client_ = node_->create_client<mavros_msgs::srv::CommandBool>(
        "mavros/cmd/arming", rmw_qos_profile_services_default, service_callback_group_);
    
    waypoint_push_client_ =
        node_->create_client<mavros_msgs::srv::WaypointPush>("mavros/mission/push");
    
    waypoint_clear_client_ =
        node_->create_client<mavros_msgs::srv::WaypointClear>("/robot_1/mavros/mission/clear");

    // 创建发布者
    gimbal_command_pub_ = node_->create_publisher<std_msgs::msg::String>("/gimbal_command", 10);
    hold_active_pub_ = node_->create_publisher<behavior_tree_msgs::msg::Active>(
        "/behavior/hold_active", 10);
}

void MavrosAdapter::set_mode(const std::string& mode, 
                             SetModeCallback callback,
                             const std::string& logger_name) {
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->base_mode = 0;
    request->custom_mode = mode;

    if (!set_mode_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] Set mode service not available", logger_name.c_str());
        callback(false);
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] Sending async set_mode request for: %s", 
               logger_name.c_str(), mode.c_str());

    // 使用 async_send_request 的回调方式（与 disarm_vehicle 一致）
    auto result = set_mode_client_->async_send_request(
        request,
        [this, callback, logger_name, mode](
            rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
            if (future.get()->mode_sent) {
                RCLCPP_INFO(node_->get_logger(), "[%s] Successfully switched to mode: %s", 
                           logger_name.c_str(), mode.c_str());
                callback(true);
            } else {
                RCLCPP_ERROR(node_->get_logger(), "[%s] Failed to switch to mode: %s", 
                            logger_name.c_str(), mode.c_str());
                callback(false);
            }
        });
    
    RCLCPP_INFO(node_->get_logger(), "[%s] Async set_mode request sent", 
               logger_name.c_str());
}

void MavrosAdapter::arm_vehicle(ArmCallback callback,
                                const std::string& logger_name) {
    RCLCPP_INFO(node_->get_logger(), "[%s] Attempting to arm the vehicle", logger_name.c_str());
    
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = true;

    if (!arm_disarm_command_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] Arming service not available", logger_name.c_str());
        callback(false);
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] Sending async arming request", logger_name.c_str());

    // 使用 async_send_request 的回调方式（与 disarm_vehicle 一致）
    auto result = arm_disarm_command_client_->async_send_request(
        request,
        [this, callback, logger_name](
            rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
            RCLCPP_INFO(node_->get_logger(), "[%s] Arming callback triggered!", logger_name.c_str());
            if (future.get()->success) {
                RCLCPP_INFO(node_->get_logger(), "[%s] Arming command succeeded", 
                           logger_name.c_str());
                callback(true);
            } else {
                RCLCPP_ERROR(node_->get_logger(), "[%s] Arming command failed", 
                            logger_name.c_str());
                callback(false);
            }
        });
        
    RCLCPP_INFO(node_->get_logger(), "[%s] Async arming request sent", 
               logger_name.c_str());
}

void MavrosAdapter::disarm_vehicle(ArmCallback callback,
                                   const std::string& logger_name) {
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = false;

    if (!arm_disarm_command_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] Disarming service not available", 
                    logger_name.c_str());
        callback(false);
        return;
    }

    auto result = arm_disarm_command_client_->async_send_request(
        request,
        [this, callback, logger_name](
            rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
            if (future.get()->success) {
                RCLCPP_INFO(node_->get_logger(), "[%s] Disarming command succeeded", 
                           logger_name.c_str());
                callback(true);
            } else {
                RCLCPP_ERROR(node_->get_logger(), "[%s] Disarming command failed", 
                            logger_name.c_str());
                callback(false);
            }
        });
}

void MavrosAdapter::clear_waypoints(WaypointCallback callback,
                                    const std::string& logger_name) {
    auto clear_request = std::make_shared<mavros_msgs::srv::WaypointClear::Request>();
    
    if (!waypoint_clear_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(node_->get_logger(),
                   "[%s] Waypoint clear service not available, proceeding without clearing.",
                   logger_name.c_str());
        callback(false);
        return;
    }

    // 使用异步回调而不是同步等待
    auto response_callback = [this, callback, logger_name](
        rclcpp::Client<mavros_msgs::srv::WaypointClear>::SharedFuture future) {
        try {
            auto result = future.get();
            if (result->success) {
                RCLCPP_INFO(node_->get_logger(), "[%s] Successfully cleared previous waypoints.", 
                           logger_name.c_str());
                callback(true);
            } else {
                RCLCPP_WARN(node_->get_logger(),
                           "[%s] Failed to clear previous waypoints.",
                           logger_name.c_str());
                callback(false);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] Exception in clear_waypoints: %s", 
                        logger_name.c_str(), e.what());
            callback(false);
        }
    };

    waypoint_clear_client_->async_send_request(clear_request, response_callback);
}

void MavrosAdapter::push_waypoints(const std::vector<sensor_msgs::msg::NavSatFix>& waypoints,
                                   double target_altitude,
                                   double hold_time,
                                   const std::vector<double>& yaws,
                                   double acceptance_radius,
                                   WaypointCallback callback,
                                   const std::string& logger_name,
                                   const std::string& current_mode) {
    auto push_request = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();
    push_request->start_index = 0;

    // 检查 yaws 是否提供并匹配航点数量
    bool use_yaws = !yaws.empty() && yaws.size() == waypoints.size();
    if (!yaws.empty() && yaws.size() != waypoints.size()) {
        RCLCPP_WARN(node_->get_logger(),
                   "[%s] Yaw angles count (%zu) doesn't match waypoints count (%zu), ignoring yaws",
                   logger_name.c_str(), yaws.size(), waypoints.size());
    }

    // 添加每个航点到请求
    for (size_t i = 0; i < waypoints.size(); i++) {
        mavros_msgs::msg::Waypoint wp;
        wp.frame = mavros_msgs::msg::Waypoint::FRAME_GLOBAL_REL_ALT;
        wp.command = 16;           // MAV_CMD_NAV_WAYPOINT
        wp.is_current = (i == 0);  // 只有第一个航点是当前航点
        wp.autocontinue = true;
        wp.param1 = hold_time;
        wp.param2 = acceptance_radius;
        wp.param3 = 0.0;
        wp.param4 = use_yaws ? yaws[i] : NAN;
        wp.x_lat = waypoints[i].latitude;
        wp.y_long = waypoints[i].longitude;
        wp.z_alt = target_altitude;
        
        RCLCPP_INFO(node_->get_logger(), "[%s] yaw=%f", logger_name.c_str(), wp.param4);
        push_request->waypoints.push_back(wp);
    }

    if (!waypoint_push_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] Waypoint push service not available", 
                    logger_name.c_str());
        callback(false);
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] Pushing %zu waypoints", 
               logger_name.c_str(), waypoints.size());

    auto push_future = waypoint_push_client_->async_send_request(
        push_request,
        [this, callback, logger_name, waypoints_count = waypoints.size(), current_mode](
            rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedFuture future) {
            if (future.valid()) {
                auto result = future.get();
                if (!result->success) {
                    RCLCPP_ERROR(node_->get_logger(), "[%s] Failed to push waypoints", 
                                logger_name.c_str());
                    callback(false);
                } else {
                    RCLCPP_INFO(node_->get_logger(), "[%s] Successfully pushed %zu waypoints",
                               logger_name.c_str(), waypoints_count);
                    
                    // 如果当前模式已经是 AUTO.MISSION，直接回调成功
                    // 否则需要在外部设置模式后再回调成功
                    if (current_mode == "AUTO.MISSION") {
                        callback(true);
                    } else {
                        // 需要切换模式，由外部处理
                        callback(true);
                    }
                }
            } else {
                RCLCPP_ERROR(node_->get_logger(), "[%s] Waypoint push future invalid", 
                            logger_name.c_str());
                callback(false);
            }
        });
}

void MavrosAdapter::publish_gimbal_command(const std::string& command, int count) {
    auto cmd_msg = std_msgs::msg::String();
    cmd_msg.data = command;
    
    for (int i = 0; i < count; i++) {
        gimbal_command_pub_->publish(cmd_msg);
    }
    
    RCLCPP_INFO(node_->get_logger(), "Sent %s command to gimbal (%d times)", 
               command.c_str(), count);
}

void MavrosAdapter::publish_hold_active(int id) {
    auto active_msg = behavior_tree_msgs::msg::Active();
    active_msg.active = true;
    active_msg.id = id;
    
    hold_active_pub_->publish(active_msg);
    RCLCPP_INFO(node_->get_logger(), "Published hold_active message with id=%d", id);
}


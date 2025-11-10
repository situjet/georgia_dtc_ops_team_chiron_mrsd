#include "action_handlers/navigate_to_waypoint_handler.hpp"
#include "mavros_adapter_interface.hpp"

#include <behavior_tree/behavior_tree.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

NavigateToWaypointHandler::NavigateToWaypointHandler(
    std::shared_ptr<IMavrosAdapter> mavros_adapter,
    rclcpp::Logger logger)
    : mavros_adapter_(mavros_adapter),
      logger_(logger),
      target_lat_(0.0),
      target_lon_(0.0),
      target_alt_(10.0) {}

void NavigateToWaypointHandler::set_target_waypoint(double lat, double lon, double alt) {
    target_lat_ = lat;
    target_lon_ = lon;
    target_alt_ = alt;
    
    RCLCPP_INFO(logger_, 
                "[Navigate to Waypoint] Target waypoint set: lat=%.6f, lon=%.6f, alt=%.1fm",
                target_lat_, target_lon_, target_alt_);
}

void NavigateToWaypointHandler::get_target_waypoint(double& lat, double& lon, double& alt) const {
    lat = target_lat_;
    lon = target_lon_;
    alt = target_alt_;
}

bool NavigateToWaypointHandler::check_safety(const Context& ctx) {
    // 检查 GPS 是否有效
    if (ctx.current_latitude == 0.0 && ctx.current_longitude == 0.0) {
        RCLCPP_ERROR(logger_, "[Navigate to Waypoint] Invalid current GPS position");
        return false;
    }

    // 检查目标航点是否有效
    if (target_lat_ == 0.0 && target_lon_ == 0.0) {
        RCLCPP_ERROR(logger_, "[Navigate to Waypoint] Invalid target waypoint (0.0, 0.0)");
        return false;
    }

    // 检查目标高度是否合理
    if (target_alt_ < MIN_ALTITUDE || target_alt_ > MAX_ALTITUDE) {
        RCLCPP_ERROR(logger_, 
                    "[Navigate to Waypoint] Invalid target altitude: %.2fm (must be between %.1f and %.1f)",
                    target_alt_, MIN_ALTITUDE, MAX_ALTITUDE);
        return false;
    }

    RCLCPP_INFO(logger_, "[Navigate to Waypoint] Safety checks passed");
    return true;
}

void NavigateToWaypointHandler::on_activated(bt::Action* action, const Context& ctx) {
    RCLCPP_INFO(logger_, 
                "[Navigate to Waypoint] Navigating to target: (%.6f, %.6f) at %.1fm altitude",
                target_lat_, target_lon_, target_alt_);

    // 创建单个航点
    sensor_msgs::msg::NavSatFix waypoint;
    waypoint.latitude = target_lat_;
    waypoint.longitude = target_lon_;
    waypoint.altitude = target_alt_;

    std::vector<sensor_msgs::msg::NavSatFix> waypoints = {waypoint};

    // 首先清除之前的任务
    mavros_adapter_->clear_waypoints(
        [this, action, waypoints, ctx](bool clear_success) {
            if (!clear_success) {
                RCLCPP_WARN(logger_, 
                           "[Navigate to Waypoint] Failed to clear waypoints, proceeding anyway");
            }
            
            RCLCPP_INFO(logger_, "[Navigate to Waypoint] Pushing waypoint to flight controller");
            
            // 推送航点
            mavros_adapter_->push_waypoints(
                waypoints,
                target_alt_,
                HOLD_TIME,
                {},  // 不使用 yaws
                ACCEPTANCE_RADIUS,
                [this, action, ctx](bool push_success) {
                    if (push_success) {
                        RCLCPP_INFO(logger_, "[Navigate to Waypoint] Successfully pushed waypoint");
                        
                        // 如果当前模式不是 AUTO.MISSION，需要切换模式
                        if (ctx.current_mode != "AUTO.MISSION") {
                            RCLCPP_INFO(logger_, 
                                       "[Navigate to Waypoint] Switching to AUTO.MISSION mode");
                            
                            mavros_adapter_->set_mode(
                                "AUTO.MISSION",
                                [this, action](bool mode_success) {
                                    if (mode_success) {
                                        action->set_success();
                                        RCLCPP_INFO(logger_, 
                                                   "[Navigate to Waypoint] Navigation started successfully");
                                    } else {
                                        action->set_failure();
                                        RCLCPP_ERROR(logger_, 
                                                    "[Navigate to Waypoint] Failed to switch to MISSION mode");
                                    }
                                },
                                "Navigate to Waypoint"
                            );
                        } else {
                            action->set_success();
                            RCLCPP_INFO(logger_, 
                                       "[Navigate to Waypoint] Navigation started (already in MISSION mode)");
                        }
                    } else {
                        action->set_failure();
                        RCLCPP_ERROR(logger_, "[Navigate to Waypoint] Failed to push waypoint");
                    }
                },
                "Navigate to Waypoint",
                ctx.current_mode
            );
        },
        "Navigate to Waypoint"
    );
}


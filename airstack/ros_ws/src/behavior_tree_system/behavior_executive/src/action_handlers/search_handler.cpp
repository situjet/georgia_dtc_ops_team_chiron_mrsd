#include "action_handlers/search_handler.hpp"
#include "mavros_adapter_interface.hpp"

#include <behavior_tree/behavior_tree.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <cmath>

SearchHandler::SearchHandler(
    std::shared_ptr<IMavrosAdapter> mavros_adapter,
    rclcpp::Logger logger)
    : mavros_adapter_(mavros_adapter),
      logger_(logger),
      target_position_({0.0, 0.0}),
      use_precise_position_(false) {}

void SearchHandler::set_target_position(const std::array<double, 2>& casualty_position, bool use_precise) {
    target_position_ = casualty_position;
    use_precise_position_ = use_precise;
    
    RCLCPP_INFO(logger_, 
                "[Search] Target position set: lat=%.6f, lon=%.6f (%s)",
                target_position_[0], target_position_[1],
                use_precise ? "PRECISE" : "RAY-CASTING ESTIMATE");
}

const std::array<double, 2>& SearchHandler::get_target_position() const {
    return target_position_;
}

bool SearchHandler::check_safety(const Context& ctx) {
    // 检查 GPS 是否有效
    if (ctx.current_latitude == 0.0 && ctx.current_longitude == 0.0) {
        RCLCPP_ERROR(logger_, "[Search] Invalid current GPS position");
        return false;
    }

    // 检查目标位置是否有效
    if (target_position_[0] == 0.0 && target_position_[1] == 0.0) {
        RCLCPP_ERROR(logger_, "[Search] Invalid target position (0.0, 0.0)");
        return false;
    }

    RCLCPP_INFO(logger_, "[Search] Safety checks passed");
    return true;
}

std::vector<sensor_msgs::msg::NavSatFix> SearchHandler::generate_circle_waypoints(
    double center_lat, double center_lon, double radius,
    int num_points, double altitude) {
    
    std::vector<sensor_msgs::msg::NavSatFix> waypoints;

    // Constants for conversion between meters and lat/lon
    const double meters_per_lat = 111111.0;  // 1 degree lat is ~111111 meters
    const double meters_per_lon = 111111.0 * std::cos(center_lat * M_PI / 180.0);

    // Generate points around a circle
    for (int i = 0; i < num_points; i++) {
        double angle = 2.0 * M_PI * i / num_points;
        
        // Calculate offset in meters
        double x_offset = radius * std::cos(angle);
        double y_offset = radius * std::sin(angle);

        // Convert to lat/lon
        sensor_msgs::msg::NavSatFix wp;
        wp.latitude = center_lat + (y_offset / meters_per_lat);
        wp.longitude = center_lon + (x_offset / meters_per_lon);
        wp.altitude = altitude;

        waypoints.push_back(wp);
    }

    return waypoints;
}

void SearchHandler::on_activated(bt::Action* action, const Context& ctx) {
    std::string location_type = use_precise_position_ ? 
        "PRECISE TARGET GPS LOCATION" : "RAY-CASTING ESTIMATED GPS LOCATION";
    
    RCLCPP_INFO(logger_, 
                "[Search] Starting search action. Using %s for search center",
                location_type.c_str());
    RCLCPP_INFO(logger_, 
                "[Search] Searching at %.6f, %.6f with altitude %.2f m",
                target_position_[0], target_position_[1], SEARCH_ALTITUDE);

    // 生成圆形搜索航点
    auto search_waypoints = generate_circle_waypoints(
        target_position_[0], 
        target_position_[1], 
        SEARCH_RADIUS,
        PERIMETER_POINTS, 
        SEARCH_ALTITUDE
    );

    if (search_waypoints.empty()) {
        RCLCPP_ERROR(logger_, "[Search] Failed to generate search waypoints");
        action->set_failure();
        return;
    }

    RCLCPP_INFO(logger_, "[Search] Generated %zu circular search waypoints", 
                search_waypoints.size());

    // 推送航点
    mavros_adapter_->push_waypoints(
        search_waypoints,
        SEARCH_ALTITUDE,
        HOLD_TIME,
        {},  // 不使用 yaws
        ACCEPTANCE_RADIUS,
        [this, action, ctx](bool push_success) {
            if (push_success) {
                RCLCPP_INFO(logger_, "[Search] Successfully pushed waypoints");
                
                // 如果当前模式不是 AUTO.MISSION，需要切换模式
                if (ctx.current_mode != "AUTO.MISSION") {
                    RCLCPP_INFO(logger_, "[Search] Switching to AUTO.MISSION mode");
                    
                    mavros_adapter_->set_mode(
                        "AUTO.MISSION",
                        [this, action](bool mode_success) {
                            if (mode_success) {
                                action->set_success();
                                RCLCPP_INFO(logger_, "[Search] Search mission started successfully");
                            } else {
                                action->set_failure();
                                RCLCPP_ERROR(logger_, "[Search] Failed to switch to MISSION mode");
                            }
                        },
                        "Search"
                    );
                } else {
                    action->set_success();
                    RCLCPP_INFO(logger_, "[Search] Search mission started (already in MISSION mode)");
                }
            } else {
                action->set_failure();
                RCLCPP_ERROR(logger_, "[Search] Failed to push waypoints");
            }
        },
        "Search",
        ctx.current_mode
    );
}


#include "action_handlers/survey_handler.hpp"
#include "mavros_adapter_interface.hpp"

#include <behavior_tree/behavior_tree.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <cmath>

SurveyHandler::SurveyHandler(
    std::shared_ptr<IMavrosAdapter> mavros_adapter,
    rclcpp::Logger logger)
    : mavros_adapter_(mavros_adapter),
      logger_(logger),
      target_position_({DEFAULT_LAT, DEFAULT_LON}),
      use_precise_position_(false) {}

void SurveyHandler::set_target_position(const std::array<double, 2>& casualty_position, bool use_precise) {
    target_position_ = casualty_position;
    use_precise_position_ = use_precise;
    
    RCLCPP_INFO(logger_, 
                "[Survey] Target position set: lat=%.6f, lon=%.6f (%s)",
                target_position_[0], target_position_[1],
                use_precise ? "PRECISE" : "DEFAULT");
}

void SurveyHandler::reset_precise_flag() {
    use_precise_position_ = false;
}

bool SurveyHandler::check_safety(const Context& ctx) {
    // 检查 GPS 是否有效
    if (ctx.current_latitude == 0.0 && ctx.current_longitude == 0.0) {
        RCLCPP_ERROR(logger_, "[Survey] Invalid current GPS position");
        return false;
    }

    RCLCPP_INFO(logger_, "[Survey] Safety checks passed");
    return true;
}

std::vector<sensor_msgs::msg::NavSatFix> SurveyHandler::generate_hexagon_waypoints(
    double center_lat, double center_lon, double radius,
    double altitude, std::vector<double>& yaws) {
    
    std::vector<sensor_msgs::msg::NavSatFix> waypoints;
    yaws.clear();

    // Constants for conversion between meters and lat/lon
    const double meters_per_lat = 111111.0;  // 1 degree lat is ~111111 meters
    const double meters_per_lon = 111111.0 * std::cos(center_lat * M_PI / 180.0);

    // Hexagon parameters
    const double sixty_deg = M_PI / 3.0;
    const double theta = 0.0;  // Rotation angle

    // Generate the 6 vertices of the hexagon
    for (int i = 0; i < 6; i++) {
        // Calculate point in meters from origin
        double x = radius * std::cos(sixty_deg * i);
        double y = radius * std::sin(sixty_deg * i);

        // Apply rotation if needed
        double rotated_x = x * std::cos(theta) - y * std::sin(theta);
        double rotated_y = x * std::sin(theta) + y * std::cos(theta);

        // Convert to lat/lon offsets and add to center
        sensor_msgs::msg::NavSatFix wp;
        wp.latitude = center_lat + (rotated_y / meters_per_lat);
        wp.longitude = center_lon + (rotated_x / meters_per_lon);
        wp.altitude = altitude;

        // Calculate heading angle (decrement 60 degrees for each vertex)
        double heading_degrees = 
            std::fmod(INITIAL_HEADING - i * HEADING_DECREMENT + 360.0, 360.0);

        waypoints.push_back(wp);
        yaws.push_back(heading_degrees);
    }

    // Add the first point again to complete the loop
    if (!waypoints.empty()) {
        waypoints.push_back(waypoints[0]);
        yaws.push_back(yaws[0]);
    }

    return waypoints;
}

void SurveyHandler::on_activated(bt::Action* action, const Context& ctx) {
    double center_lat, center_lon;
    std::string location_desc;

    if (use_precise_position_) {
        center_lat = target_position_[0];
        center_lon = target_position_[1];
        location_desc = "precise casualty location";
    } else {
        center_lat = DEFAULT_LAT;
        center_lon = DEFAULT_LON;
        location_desc = "default drone location";
    }

    RCLCPP_INFO(logger_, 
                "[Survey] Survey action activated. Using %s for center.",
                location_desc.c_str());
    RCLCPP_INFO(logger_, "[Survey] Center point for the hexagon: lat=%f, lon=%f",
                center_lat, center_lon);

    // 生成六边形航点和偏航角
    std::vector<double> hexagon_yaws;
    auto hexagon_waypoints = generate_hexagon_waypoints(
        center_lat, 
        center_lon, 
        SURVEY_RADIUS,
        SURVEY_ALTITUDE,
        hexagon_yaws
    );

    if (hexagon_waypoints.empty()) {
        RCLCPP_ERROR(logger_, "[Survey] Failed to generate any hexagon waypoints");
        action->set_failure();
        return;
    }

    RCLCPP_INFO(logger_, "[Survey] Generated %zu hexagon waypoints (including loop closure)",
                hexagon_waypoints.size());

    // 发送云台锁定命令
    mavros_adapter_->publish_gimbal_command("LOCK_ON", 3);

    // 首先清除之前的任务
    mavros_adapter_->clear_waypoints(
        [this, action, hexagon_waypoints, hexagon_yaws, ctx](bool clear_success) {
            // 推送生成的航点
            mavros_adapter_->push_waypoints(
                hexagon_waypoints,
                SURVEY_ALTITUDE,
                HOLD_TIME,
                hexagon_yaws,
                ACCEPTANCE_RADIUS,
                [this, action](bool push_success) {
                    if (push_success) {
                        action->set_success();
                        RCLCPP_INFO(logger_, "[Survey] Survey mission started successfully");
                    } else {
                        action->set_failure();
                        RCLCPP_ERROR(logger_, "[Survey] Failed to push waypoints");
                    }
                },
                "Survey",
                ctx.current_mode
            );
        },
        "Survey"
    );

    // 重置精确检测标志（在使用后）
    reset_precise_flag();
}





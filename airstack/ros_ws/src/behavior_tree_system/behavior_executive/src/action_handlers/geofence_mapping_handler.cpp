#include "action_handlers/geofence_mapping_handler.hpp"
#include "mission_planning/geometry_utils.hpp"
#include "mission_planning/waypoint_generator.hpp"
#include "mavros_adapter_interface.hpp"

#include <behavior_tree/behavior_tree.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

GeofenceMappingHandler::GeofenceMappingHandler(
    std::shared_ptr<IMavrosAdapter> mavros_adapter,
    std::shared_ptr<WaypointGenerator> waypoint_generator,
    std::shared_ptr<GeometryUtils> geometry_utils,
    rclcpp::Logger logger)
    : mavros_adapter_(mavros_adapter),
      waypoint_generator_(waypoint_generator),
      geometry_utils_(geometry_utils),
      logger_(logger) {}

void GeofenceMappingHandler::set_geofence_points(
    const std::vector<std::pair<double, double>>& points) {
    geofence_points_ = points;
}

const std::vector<std::pair<double, double>>& 
GeofenceMappingHandler::get_geofence_points() const {
    return geofence_points_;
}

bool GeofenceMappingHandler::check_safety(const Context& ctx) {
    // 检查 GPS 是否有效
    if (ctx.current_latitude == 0.0 && ctx.current_longitude == 0.0) {
        RCLCPP_ERROR(logger_, "[Geofence Mapping] Invalid GPS position");
        return false;
    }

    // 检查是否有围栏点（可以使用预定义或接收的）
    // 如果没有围栏点，使用预定义的会在 on_activated 中处理
    
    // 检查目标高度是否合理
    if (ctx.target_altitude <= 0.0 || ctx.target_altitude > 120.0) {
        RCLCPP_ERROR(logger_, "[Geofence Mapping] Invalid target altitude: %.2f m", 
                    ctx.target_altitude);
        return false;
    }

    return true;
}

void GeofenceMappingHandler::on_activated(bt::Action* action, const Context& ctx) {
    RCLCPP_INFO(logger_, "[Geofence Mapping] Starting geofence mapping action from current position");

    // 获取当前位置
    double current_lat = ctx.current_latitude;
    double current_lon = ctx.current_longitude;

    // 使用接收的围栏点，如果没有则使用预定义的
    std::vector<std::pair<double, double>> geofence_points_input;
    if (!geofence_points_.empty()) {
        geofence_points_input = geofence_points_;
        RCLCPP_INFO(logger_, "[Geofence Mapping] Using %zu received geofence points",
                    geofence_points_input.size());
    } else {
        // Blair / Mil 19 预定义围栏
        geofence_points_input = {
            {40.4141646, -79.9475044},
            {40.4138379, -79.9475000},
            {40.4137503, -79.9479319},
            {40.4140025, -79.9480247}
        };
        RCLCPP_INFO(logger_, "[Geofence Mapping] Using Blair / Mil 19 predefined geofence");
    }

    // 计算凸包使围栏顺序无关
    std::vector<std::pair<double, double>> geofence_corners = 
        geometry_utils_->convex_hull(geofence_points_input);

    if (geofence_corners.size() < 3) {
        RCLCPP_ERROR(logger_, "[Geofence Mapping] Convex hull requires at least 3 points.");
        action->set_failure();
        return;
    }

    RCLCPP_INFO(logger_, "[Geofence Mapping] Computed convex hull with %zu points",
                geofence_corners.size());

    // 生成割草机模式的航点
    auto nav_waypoints = waypoint_generator_->generate_lawnmower_pattern(
        geofence_corners,
        current_lat,
        current_lon,
        FOOTPRINT_WIDTH,
        OVERLAP_PERCENTAGE,
        SAFE_DISTANCE,
        ctx.target_altitude
    );

    RCLCPP_INFO(logger_, "[Geofence Mapping] Generated %zu waypoints covering full geofence bounds",
                nav_waypoints.size());

    if (nav_waypoints.empty()) {
        RCLCPP_WARN(logger_, "[Geofence Mapping] No valid waypoints generated within the geofence.");
        action->set_failure();
        return;
    }

    // 首先清除之前的任务
    mavros_adapter_->clear_waypoints(
        [this, action, nav_waypoints, ctx](bool clear_success) {
            // 无论清除是否成功都继续推送航点
            RCLCPP_INFO(logger_, "[Geofence Mapping] Pushing %zu waypoints to flight controller",
                       nav_waypoints.size());
            
            // 推送生成的航点
            mavros_adapter_->push_waypoints(
                nav_waypoints,
                ctx.target_altitude,
                HOLD_TIME,
                {},  // 不使用 yaws
                ACCEPTANCE_RADIUS,
                [this, action, ctx](bool push_success) {
                    if (push_success) {
                        RCLCPP_INFO(logger_, "[Geofence Mapping] Successfully pushed waypoints");
                        
                        // 如果当前模式不是 AUTO.MISSION，需要切换模式
                        if (ctx.current_mode != "AUTO.MISSION") {
                            mavros_adapter_->set_mode(
                                "AUTO.MISSION",
                                [this, action](bool mode_success) {
                                    if (mode_success) {
                                        action->set_success();
                                        RCLCPP_INFO(logger_, "[Geofence Mapping] Mission started successfully");
                                    } else {
                                        action->set_failure();
                                        RCLCPP_ERROR(logger_, "[Geofence Mapping] Failed to switch to MISSION mode");
                                    }
                                },
                                "Geofence Mapping"
                            );
                        } else {
                            action->set_success();
                            RCLCPP_INFO(logger_, "[Geofence Mapping] Mission started (already in MISSION mode)");
                        }
                    } else {
                        action->set_failure();
                        RCLCPP_ERROR(logger_, "[Geofence Mapping] Failed to push waypoints");
                    }
                },
                "Geofence Mapping",
                ctx.current_mode
            );
        },
        "Geofence Mapping"
    );
}





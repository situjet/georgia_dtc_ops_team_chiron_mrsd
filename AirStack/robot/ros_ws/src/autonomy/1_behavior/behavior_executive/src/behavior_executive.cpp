#include <rmw/qos_profiles.h>  // For rmw_qos_profile_services_default

#include <airstack_msgs/srv/trajectory_mode.hpp>
#include <algorithm>  // For std::min, std::max
#include <behavior_executive.hpp>
#include <behavior_tree_msgs/msg/active.hpp>  // For the publisher in UAV_Deconflict_callback
#include <chrono>                             // For std::chrono_literals
#include <cmath>       // For std::cos, std::sin, std::sqrt, M_PI, std::asin, std::abs
#include <functional>  // For std::bind
#include <limits>      // For std::numeric_limits
#include <mavros_msgs/msg/waypoint.hpp>
#include <mavros_msgs/msg/waypoint_list.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/waypoint_clear.hpp>
#include <mavros_msgs/srv/waypoint_push.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>  // For std::vector

// #include "vision_msgs/msg/detection2_d_array.hpp"
using namespace std::chrono_literals;

BehaviorExecutive::BehaviorExecutive() : Node("behavior_executive") {
    // ========================
    // Behavior Tree Nodes
    // ========================

    // Auto Takeoff Command Expected Behavior
    // auto takeoff only if not armed
    // switch to takeoff mode, arm, wait for 2 secs, switch to hold
    auto_takeoff_commanded_condition = new bt::Condition("Auto Takeoff Commanded", this);
    armed_condition = new bt::Condition("Armed", this);
    auto_takeoff_action = new bt::Action("Request AutoTakeoff", this);

    // 2.Land Behavior
    // switch to autoland mode, land
    // land_commanded_condition = new bt::Condition("Land Commanded", this);  // rqt_gui button
    // land_action = new bt::Action("Land", this);

    // 3.AutoLand Behavior
    auto_land_commanded_condition =
        new bt::Condition("AutoLand Commanded", this);  // rqt_gui button
    auto_land_action = new bt::Action("AutoLand", this);

    // 4.Arm Behavior
    arm_commanded_condition = new bt::Condition("Arm Commanded", this);  // rqt_gui button
    arm_action = new bt::Action("Arm", this);

    // 5. Disarm Behavior
    disarm_commanded_condition = new bt::Condition("Disarm Commanded", this);  // rqt_gui button
    disarm_action = new bt::Action("Disarm", this);

    // 6. E-Stop Behavior
    e_stop_commanded_condition = new bt::Condition("EStop Commanded", this);
    hold_action = new bt::Action("Hold", this);

    survey_commanded_condition = new bt::Condition("Survey Commanded", this);
    survey_action = new bt::Action("Survey", this);

    // 9. Geofence Mapping Behavior
    geofence_mapping_commanded_condition =
        new bt::Condition("Geofence Mapping Commanded", this);  // rqt_gui button
    geofence_mapping_action = new bt::Action("Geofence Mapping", this);

    // 10. Search Behavior
    search_commanded_condition = new bt::Condition("Search Commanded", this);
    search_action = new bt::Action("Search", this);

    conditions.push_back(auto_takeoff_commanded_condition);
    conditions.push_back(armed_condition);
    conditions.push_back(survey_commanded_condition);

    conditions.push_back(arm_commanded_condition);
    conditions.push_back(geofence_mapping_commanded_condition);
    conditions.push_back(disarm_commanded_condition);
    conditions.push_back(e_stop_commanded_condition);
    conditions.push_back(auto_land_commanded_condition);
    conditions.push_back(search_commanded_condition);
    // actions

    actions.push_back(arm_action);
    actions.push_back(auto_takeoff_action);
    actions.push_back(hold_action);
    actions.push_back(disarm_action);
    actions.push_back(auto_land_action);

    actions.push_back(geofence_mapping_action);
    actions.push_back(search_action);

    // Subscribers Callback
    gps_deconflict_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/gps_list", 10,
        std::bind(&BehaviorExecutive::UAV_Deconflict_callback, this, std::placeholders::_1));
    behavior_tree_commands_sub =
        this->create_subscription<behavior_tree_msgs::msg::BehaviorTreeCommands>(
            "behavior_tree_commands", 1,
            std::bind(&BehaviorExecutive::bt_commands_callback, this, std::placeholders::_1));
    state_sub = this->create_subscription<mavros_msgs::msg::State>(
        "mavros/state", rclcpp::QoS(10).best_effort(),
        std::bind(&BehaviorExecutive::state_callback, this, std::placeholders::_1));
    current_gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "mavros/global_position/global", rclcpp::QoS(10).best_effort(),
        std::bind(&BehaviorExecutive::current_gps_callback, this, std::placeholders::_1));
    relative_altitude_sub = this->create_subscription<std_msgs::msg::Float64>(
        "mavros/global_position/rel_alt", rclcpp::QoS(10).best_effort(),
        std::bind(&BehaviorExecutive::relative_altitude_callback, this, std::placeholders::_1));
    target_waypoint_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "target_waypoint", rclcpp::QoS(10).best_effort(),
        std::bind(&BehaviorExecutive::target_waypoint_callback, this, std::placeholders::_1));
    set_target_altitude_sub = this->create_subscription<std_msgs::msg::Float64>(
        "set_target_altitude", rclcpp::QoS(10).best_effort(),
        std::bind(&BehaviorExecutive::set_target_altitude_callback, this, std::placeholders::_1));

    geofence_sub = this->create_subscription<mavros_msgs::msg::WaypointList>(
        "/robot_1/mavros/geofence/fences", rclcpp::QoS(10).durability_volatile().transient_local(),
        std::bind(&BehaviorExecutive::geofence_callback, this, std::placeholders::_1));
    target_gps_list_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/target_gps_list", 10,
        std::bind(&BehaviorExecutive::target_gps_list_callback, this, std::placeholders::_1));
    target_gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/target_gps", 10,
        std::bind(&BehaviorExecutive::target_gps_callback, this, std::placeholders::_1));
    gimbal_command_pub = this->create_publisher<std_msgs::msg::String>("/gimbal_command", 10);

    // Service Clients
    service_callback_group =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    set_mode_client = this->create_client<mavros_msgs::srv::SetMode>(
        "mavros/set_mode", rmw_qos_profile_services_default, service_callback_group);
    arm_disarm_command_client = this->create_client<mavros_msgs::srv::CommandBool>(
        "mavros/cmd/arming", rmw_qos_profile_services_default, service_callback_group);
    takeoff_command_client = this->create_client<mavros_msgs::srv::CommandTOL>(
        "mavros/cmd/takeoff", rmw_qos_profile_services_default, service_callback_group);
    land_command_client = this->create_client<mavros_msgs::srv::CommandTOL>(
        "mavros/cmd/land", rmw_qos_profile_services_default, service_callback_group);

    waypoint_push_client =
        this->create_client<mavros_msgs::srv::WaypointPush>("mavros/mission/push");
    waypoint_clear_client =
        this->create_client<mavros_msgs::srv::WaypointClear>("/robot_1/mavros/mission/clear");

    // ==============
    // Control Timer
    // ==============
    timer = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(1. / 20.),
                                 std::bind(&BehaviorExecutive::timer_callback, this));

    current_mode = "";
    // Add these class members at the top with other variables
    // Terrain Altitude
    terrain_altitude = 0.0;           // meters
    current_relative_altitude = 0.0;  // meters
    current_latitude = 0.0;           // degrees
    current_longitude = 0.0;          // degrees

    // Target Altitudes
    target_takeoff_altitude = 0.0;  // meters
    target_landing_altitude = 0.0;  // meters
    target_ascend_altitude = 9.0;   // meters

    // Tolerances
    takeoff_altitude_threshold = 0.5;  // 0.5m tolerance
    landing_altitude_threshold = 0.2;  // 0.5m tolerance
    ascend_altitude_threshold = 0.5;   // 0.5m tolerance

    // Target Waypoint - initialize individual elements instead of using initializer list
    target_waypoint.latitude = 0.0;
    target_waypoint.longitude = 0.0;

    // Geofence Mapping
    // geofence_mapping_radius = 10.0;  // meters

    // Initialize hexagon navigation variables
    current_waypoint_index = 0;
    is_hexagon_in_progress = false;
    hexagon_waypoints.clear();
    casualty_gps_position = {0.0, 0.0};
    precise_casualty_gps_position = {0.0, 0.0};
    hexagon_timer = nullptr;

    // // Added for the new drone_pose_callback
    // current_heading_radians = 0.0;  // Heading in radians in ENU frame
    // current_heading_degrees = 0.0;  // Heading in degrees [0, 360)

    hold_active_id = 0;  // used to publish different ids

    // initialize search related variables
    is_search_in_progress = false;
    search_waypoints.clear();
    search_current_waypoint_index = 0;
    search_timer = nullptr;

    precise_casualty_detected = false;

    // 在构造函数中创建订阅
    precise_target_gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/precise_target_gps", rclcpp::QoS(10).best_effort(),
        std::bind(&BehaviorExecutive::precise_target_gps_callback, this, std::placeholders::_1));

    min_confidence = std::numeric_limits<float>::max();  // initialize to max float
}

// ACTIONS CALLBACK
void BehaviorExecutive::timer_callback() {
    // 1. Auto Takeoff
    // DEBUGGING NOTE:
    // PX4 (v1.15.4) Auto Takeoff Behavior
    // Vehicle will takeoff and check if it has reached target altitude
    // If it has reached target altitude, it will switch to loiter mode
    // If it has not reached target altitude, it will continue to takeoff

    // If in reality the takeoff was not successful, the vehicle likely AutoLand itself
    // GO TO PX4 Log and logs.px4.io to check for error messages
    // error1: critical battery level, RTL triggered
    // Go To QGC MIS_TAKEOFF_ALT to set takeoff altitude

    if (auto_takeoff_action->is_active()) {
        auto_takeoff_action->set_running();
        if (auto_takeoff_action->active_has_changed()) {
            // Set terrain altitude to current relative altitude
            terrain_altitude = current_relative_altitude;
            RCLCPP_INFO(this->get_logger(), "Setting terrain altitude to: %f meters",
                        terrain_altitude);
            // Adds offset to target_ascend_altitude
            target_ascend_altitude = terrain_altitude + target_ascend_altitude;
            RCLCPP_INFO(this->get_logger(), "Target Flight Altitude Updated: %f meters",
                        target_ascend_altitude);

            // Set to takeoff mode
            set_mode(auto_takeoff_action, set_mode_client, "AUTO.TAKEOFF");

            auto_takeoff_action->set_success();
        }
    }

    // 2. Arm
    if (arm_action->is_active()) {
        if (arm_action->active_has_changed()) {
            RCLCPP_INFO(this->get_logger(), "Attempting to arm the vehicle");
            auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            request->value = true;

            if (!arm_disarm_command_client->wait_for_service(std::chrono::seconds(2))) {
                RCLCPP_ERROR(this->get_logger(), "Arming service not available");
                arm_action->set_failure();
                return;
            }

            auto result = arm_disarm_command_client->async_send_request(
                request,
                [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
                    if (future.get()->success) {
                        RCLCPP_INFO(this->get_logger(), "Arming command succeeded");
                        arm_action->set_success();
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Arming command failed");
                        arm_action->set_failure();
                    }
                });
        }
    }

    // 3. Disarm
    if (disarm_action->is_active()) {
        disarm_action->set_running();
        if (disarm_action->active_has_changed()) {
            auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            request->value = false;

            if (!arm_disarm_command_client->wait_for_service(std::chrono::seconds(2))) {
                RCLCPP_ERROR(this->get_logger(), "Disarming service not available");
                disarm_action->set_failure();
                return;
            }

            auto result = arm_disarm_command_client->async_send_request(
                request,
                [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
                    if (future.get()->success) {
                        RCLCPP_INFO(this->get_logger(), "Disarming command succeeded");
                        disarm_action->set_success();
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Disarming command failed");
                        disarm_action->set_failure();
                    }
                });
        }
    }

    // 4. Auto Land
    if (auto_land_action->is_active()) {
        auto_land_action->set_running();
        if (auto_land_action->active_has_changed()) {
            if (this->current_mode != "AUTO.LAND") {
                set_mode(auto_land_action, set_mode_client, "AUTO.LAND");
            }
        }
    }

    // 5. E-Stop
    if (hold_action->is_active()) {
        hold_action->set_running();
        if (hold_action->active_has_changed()) {
            if (this->current_mode == "AUTO.LOITER") {
                RCLCPP_INFO(this->get_logger(), "Already in AUTO.LOITER mode");
                hold_action->set_success();
                return;
            }

            set_mode(hold_action, set_mode_client, "AUTO.LOITER");
            // print hold_action status
            // RCLCPP_INFO(this->get_logger(), "Hold action status: %d", hold_action->is_success());
        }
    }

    // 6. trying geofence mapping
    if (geofence_mapping_action->is_active()) {
        geofence_mapping_action->set_running();
        if (geofence_mapping_action->active_has_changed()) {
            RCLCPP_INFO(this->get_logger(),
                        "Starting geofence mapping action from current position");

            // Get drone's current position
            double current_lat = current_latitude;
            double current_lon = current_longitude;

            // Use the received geofence points if available, otherwise use the hardcoded ones
            std::vector<std::pair<double, double>> geofence_points_input;
            if (!geofence_points.empty()) {
                geofence_points_input = geofence_points;
                RCLCPP_INFO(this->get_logger(), "Using %zu received geofence points",
                            geofence_points_input.size());
            } else {
                // Blair / Mil 19
                geofence_points_input = {
                    //{40.4253101, -79.9544781},
                    {40.4252678, -79.9545928},
                    {40.4254007, -79.9542703},
                    {40.4252553, -79.9541187},
                    {40.4250965, -79.9543229}  //{40.4251402, -79.9542884}
                };
                RCLCPP_INFO(this->get_logger(), "Using Blair / Mil 19 predefined geofence");
                // Hawkins Field
                // geofence_points_input = {{40.4781273, -79.8926137},
                //                          {40.4779946, -79.8923089},
                //                          {40.4777228, -79.8924916},
                //                          {40.4778669, -79.8927944}};
                // RCLCPP_INFO(this->get_logger(), "Using Hawkins Field predefined geofence");
            }
            // Compute the convex hull to make the geofence order-agnostic
            std::vector<std::pair<double, double>> geofence_corners =
                convex_hull(geofence_points_input);

            if (geofence_corners.size() < 3) {
                RCLCPP_ERROR(this->get_logger(), "Convex hull requires at least 3 points.");
                geofence_mapping_action->set_failure();
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Computed convex hull with %zu points",
                        geofence_corners.size());

            // Calculate center of the convex hull
            double center_lat = 0.0;
            double center_lon = 0.0;
            for (const auto& corner : geofence_corners) {
                center_lat += corner.first;
                center_lon += corner.second;
            }
            center_lat /= geofence_corners.size();
            center_lon /= geofence_corners.size();

            double meters_per_deg_lat = 111111.0;  // Standardized constant
            double meters_per_deg_lon =
                111111.0 * std::cos(center_lat * M_PI / 180.0);  // Use standardized constant

            double footprint_width = 3.5;
            double overlap_percentage = 10.0;
            double safe_distance = 2.0;  // Safe distance from the geofence boundary in meters

            // Calculate safe distance in degrees
            double safe_dist_lat = safe_distance / meters_per_deg_lat;
            double safe_dist_lon = safe_distance / meters_per_deg_lon;

            // Calculate bounds
            double min_lat = std::numeric_limits<double>::max();
            double max_lat = std::numeric_limits<double>::lowest();
            double min_lon = std::numeric_limits<double>::max();
            double max_lon = std::numeric_limits<double>::lowest();

            for (const auto& corner : geofence_corners) {
                min_lat = std::min(min_lat, corner.first);
                max_lat = std::max(max_lat, corner.first);
                min_lon = std::min(min_lon, corner.second);
                max_lon = std::max(max_lon, corner.second);
            }

            // Calculate step sizes based on footprint and overlap
            double footprint_width_deg_lat = footprint_width / meters_per_deg_lat;
            double footprint_width_deg_lon = footprint_width / meters_per_deg_lon;

            double overlap_fraction = overlap_percentage / 100.0;
            double step_size_lat = footprint_width_deg_lat * (1.0 - overlap_fraction);
            double step_size_lon = footprint_width_deg_lon * (1.0 - overlap_fraction);

            // Ensure step sizes are positive
            if (step_size_lat <= 0 || step_size_lon <= 0) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Invalid step size calculated (<= 0). Check footprint/overlap parameters.");
                geofence_mapping_action->set_failure();
                return;
            }

            // Calculate number of passes based on the full bounding box extent
            int num_passes_lon =
                std::max(1, static_cast<int>(std::ceil((max_lon - min_lon) / step_size_lon)));
            int num_passes_lat =
                std::max(1, static_cast<int>(std::ceil((max_lat - min_lat) / step_size_lat)));

            // Snap current position to nearest latitude step on the grid covering the full bounds
            double snapped_lat =
                std::round((current_lat - min_lat) / step_size_lat) * step_size_lat + min_lat;
            int start_lat_idx = std::round((snapped_lat - min_lat) / step_size_lat);
            // Clamp index to be within valid grid range [0, num_passes_lat - 1]
            start_lat_idx = std::max(0, std::min(start_lat_idx, num_passes_lat - 1));

            // Snap current position to nearest longitude step on the grid covering the full bounds
            std::vector<double> lon_array;
            for (int j = 0; j < num_passes_lon; j++) {
                lon_array.push_back(min_lon + j * step_size_lon);
            }
            int start_lon_idx = 0;  // Default if lon_array is empty
            if (!lon_array.empty()) {
                double closest_lon = lon_array[0];
                double min_diff = std::abs(current_lon - closest_lon);
                for (size_t j = 1; j < lon_array.size(); ++j) {
                    double diff = std::abs(current_lon - lon_array[j]);
                    if (diff < min_diff) {
                        min_diff = diff;
                        closest_lon = lon_array[j];
                        start_lon_idx = j;
                    }
                }
            }
            // Clamp index to be within valid grid range [0, num_passes_lon - 1]
            start_lon_idx = std::max(0, std::min(start_lon_idx, num_passes_lon - 1));

            std::vector<std::pair<double, double>> waypoints;

            // Generate waypoints starting near the current position using lawnmower pattern
            // The loops will now iterate based on the full grid dimensions
            bool going_right = true;  // Initial direction (can be refined)
            std::vector<bool> row_processed(num_passes_lat, false);

            // Process the starting latitude row
            if (start_lat_idx >= 0 && start_lat_idx < num_passes_lat) {
                double lat = min_lat + start_lat_idx * step_size_lat;
                std::vector<std::pair<double, double>> current_row_waypoints;
                if (going_right) {
                    // Start from snapped longitude index and go right
                    for (int j = start_lon_idx; j < num_passes_lon; j++) {
                        std::pair<double, double> wp = {lat, min_lon + j * step_size_lon};
                        if (is_point_in_fence(wp, geofence_corners) &&
                            is_point_safe_distance(wp, geofence_corners, safe_dist_lat,
                                                   safe_dist_lon))
                            current_row_waypoints.push_back(wp);
                    }
                    // Then sweep back left for remaining points in this row if any
                    for (int j = start_lon_idx - 1; j >= 0; --j) {
                        std::pair<double, double> wp = {lat, min_lon + j * step_size_lon};
                        if (is_point_in_fence(wp, geofence_corners) &&
                            is_point_safe_distance(wp, geofence_corners, safe_dist_lat,
                                                   safe_dist_lon))
                            current_row_waypoints.push_back(wp);
                    }
                } else {
                    // Start from snapped longitude index and go left
                    for (int j = start_lon_idx; j >= 0; j--) {
                        std::pair<double, double> wp = {lat, min_lon + j * step_size_lon};
                        if (is_point_in_fence(wp, geofence_corners) &&
                            is_point_safe_distance(wp, geofence_corners, safe_dist_lat,
                                                   safe_dist_lon))
                            current_row_waypoints.push_back(wp);
                    }
                    // Then sweep back right for remaining points if any
                    for (int j = start_lon_idx + 1; j < num_passes_lon; ++j) {
                        std::pair<double, double> wp = {lat, min_lon + j * step_size_lon};
                        if (is_point_in_fence(wp, geofence_corners) &&
                            is_point_safe_distance(wp, geofence_corners, safe_dist_lat,
                                                   safe_dist_lon))
                            current_row_waypoints.push_back(wp);
                    }
                }
                waypoints.insert(waypoints.end(), current_row_waypoints.begin(),
                                 current_row_waypoints.end());
                row_processed[start_lat_idx] = true;
                going_right = !going_right;  // Flip direction for the next row
            }

            // Process rows above the starting row
            for (int i = start_lat_idx + 1; i < num_passes_lat; i++) {
                if (row_processed[i]) continue;
                double lat = min_lat + i * step_size_lat;
                std::vector<std::pair<double, double>> current_row_waypoints;
                if (going_right) {
                    for (int j = 0; j < num_passes_lon; j++) {
                        std::pair<double, double> wp = {lat, min_lon + j * step_size_lon};
                        if (is_point_in_fence(wp, geofence_corners) &&
                            is_point_safe_distance(wp, geofence_corners, safe_dist_lat,
                                                   safe_dist_lon))
                            current_row_waypoints.push_back(wp);
                    }
                } else {
                    for (int j = num_passes_lon - 1; j >= 0; j--) {
                        std::pair<double, double> wp = {lat, min_lon + j * step_size_lon};
                        if (is_point_in_fence(wp, geofence_corners) &&
                            is_point_safe_distance(wp, geofence_corners, safe_dist_lat,
                                                   safe_dist_lon))
                            current_row_waypoints.push_back(wp);
                    }
                }
                if (!current_row_waypoints.empty()) {
                    waypoints.insert(waypoints.end(), current_row_waypoints.begin(),
                                     current_row_waypoints.end());
                    row_processed[i] = true;
                    going_right = !going_right;  // Flip direction
                }
            }

            // Process rows below the starting row
            // Reset 'going_right' based on the direction needed for the first row below
            // start_lat_idx Find the direction used for the last row processed (either
            // start_lat_idx or the highest row above it) Let's assume the 'going_right' state after
            // processing rows above is correct for the next unprocessed row.

            for (int i = start_lat_idx - 1; i >= 0; i--) {
                if (row_processed[i]) continue;
                double lat = min_lat + i * step_size_lat;
                std::vector<std::pair<double, double>> current_row_waypoints;
                if (going_right) {
                    for (int j = 0; j < num_passes_lon; j++) {
                        std::pair<double, double> wp = {lat, min_lon + j * step_size_lon};
                        if (is_point_in_fence(wp, geofence_corners) &&
                            is_point_safe_distance(wp, geofence_corners, safe_dist_lat,
                                                   safe_dist_lon))
                            current_row_waypoints.push_back(wp);
                    }
                } else {
                    for (int j = num_passes_lon - 1; j >= 0; j--) {
                        std::pair<double, double> wp = {lat, min_lon + j * step_size_lon};
                        if (is_point_in_fence(wp, geofence_corners) &&
                            is_point_safe_distance(wp, geofence_corners, safe_dist_lat,
                                                   safe_dist_lon))
                            current_row_waypoints.push_back(wp);
                    }
                }
                if (!current_row_waypoints.empty()) {
                    waypoints.insert(waypoints.end(), current_row_waypoints.begin(),
                                     current_row_waypoints.end());
                    row_processed[i] = true;
                    going_right = !going_right;  // Flip direction
                }
            }

            RCLCPP_INFO(this->get_logger(), "Generated %zu waypoints covering full geofence bounds",
                        waypoints.size());

            if (waypoints.empty()) {
                RCLCPP_WARN(this->get_logger(),
                            "No valid waypoints generated within the geofence.");
                geofence_mapping_action->set_failure();
                return;
            }

            std::vector<sensor_msgs::msg::NavSatFix> nav_waypoints;
            for (const auto& wp_pair : waypoints) {
                sensor_msgs::msg::NavSatFix wp;
                wp.latitude = wp_pair.first;
                wp.longitude = wp_pair.second;
                wp.altitude = target_ascend_altitude;  // Ensure altitude is set
                nav_waypoints.push_back(wp);
            }

            // Clear previous mission before pushing new one
            auto clear_request = std::make_shared<mavros_msgs::srv::WaypointClear::Request>();
            if (!waypoint_clear_client->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_WARN(this->get_logger(),
                            "Waypoint clear service not available, proceeding without clearing.");
            } else {
                auto clear_future = waypoint_clear_client->async_send_request(clear_request);
                if (clear_future.wait_for(std::chrono::seconds(2)) == std::future_status::ready &&
                    clear_future.get()->success) {
                    RCLCPP_INFO(this->get_logger(), "Successfully cleared previous waypoints.");
                } else {
                    RCLCPP_WARN(this->get_logger(),
                                "Failed to clear previous waypoints or service timed out.");
                }
            }

            // Push the generated waypoints
            push_waypoints(geofence_mapping_action, nav_waypoints, 10,
                           1.5,   // Using 0.0 hold time
                           {},    // No yaws for geofence mapping
                           0.5);  // Acceptance radius for geofence mapping
        }
    }

    // 7. Search
    if (search_action->is_active()) {
        search_action->set_running();

        if (search_action->active_has_changed()) {
            RCLCPP_INFO(this->get_logger(), "Starting search action");

            // check if target GPS data is received
            // if (target_gps.latitude == 0.0 && target_gps.longitude == 0.0) {
            //     RCLCPP_ERROR(this->get_logger(), "No target GPS data received");
            //     search_action->set_failure();
            //     return;
            // }

            // check and switch to mission mode
            if (this->current_mode != "AUTO.MISSION") {
                set_mode(search_action, set_mode_client, "AUTO.MISSION");
            }

            // cover 3m radius area
            double radius = 4.5;
            int perimeter_points = 25 - 1;  // remove the center point

            double center_lat = 0.0;
            double center_lon = 0.0;

            if (precise_casualty_detected) {
                center_lat = precise_casualty_gps_position[0];
                center_lon = precise_casualty_gps_position[1];
                RCLCPP_INFO(this->get_logger(),
                            "[SEARCH INFO] USING PRECISE TARGET GPS LOCATION FOR SEARCH CENTER");
            } else {
                center_lat = casualty_gps_position[0];
                center_lon = casualty_gps_position[1];
                RCLCPP_INFO(
                    this->get_logger(),
                    "[SEARCH INFO] USING RAY-CASTING ESTIMATED GPS LOCATION FOR SEARCH CENTER");
            }

            search_waypoints = generate_circle_waypoints(center_lat, center_lon, radius,
                                                         perimeter_points, target_ascend_altitude);
            // std::vector<sensor_msgs::msg::NavSatFix> search_waypoints;
            // sensor_msgs::msg::NavSatFix wp;
            // wp.latitude = center_lat;
            // wp.longitude = center_lon;

            // wp.altitude = 10;
            // search_waypoints.push_back(wp);

            if (!search_waypoints.empty()) {
                RCLCPP_INFO(this->get_logger(),
                            "[SEARCH INFO] SEARCHING AT %.6f, %.6f with altitude %.2f m",
                            center_lat, center_lon, 10);

                // push all waypoints
                push_waypoints(search_action, search_waypoints, 10, 2.0, {},
                               0.5);  // push all waypoints

                // RCLCPP_INFO(this->get_logger(),
                //             "Target search waypoint pushed.");
            } else {
                // This case should ideally not be reached now unless Waypoint creation fails
                RCLCPP_ERROR(this->get_logger(), "[ERROR]: FAILED CREATING SEARCH WAYPOINT");
                search_action->set_failure();
            }
        }
    }

    // 8. Survey
    if (survey_action->is_active()) {
        survey_action->set_running();  // Keep this outside, action is running

        // Only generate and push waypoints when the action first becomes active
        if (survey_action->active_has_changed()) {
            RCLCPP_INFO(this->get_logger(),
                        "Survey action activated. Generating hexagon waypoints.");

            double center_lat = 0.0;
            double center_lon = 0.0;

            if (precise_casualty_detected) {
                center_lat = precise_casualty_gps_position[0];
                center_lon = precise_casualty_gps_position[1];
                RCLCPP_INFO(this->get_logger(),
                            "Using precise casualty location for survey center.");
            } else {
                center_lat = 40.4779091;   // current_latitude;
                center_lon = -79.8927102;  // current_longitude;
                RCLCPP_INFO(this->get_logger(), "Using current drone location for survey center.");
            }

            // Now center_lat and center_lon are accessible here
            RCLCPP_INFO(this->get_logger(), "Center point for the hexagon: lat=%f, lon=%f",
                        center_lat, center_lon);


            // Constants for conversion between meters and lat/lon
            // These are approximations for the given latitude
            // calculate the earth radius
            const double meters_per_lat = 111111.0;  // 1 degree lat is ~111111 meters
            const double meters_per_lon =
                111111.0 * std::cos(center_lat * M_PI / 180.0);  // 1 degree lon depends on latitude

            // Generate hexagon waypoints (radius meters from center)
            hexagon_waypoints.clear();  // Clear any previous waypoints
            const double sixty_deg = M_PI / 3.0;
            const double radius = 4.5;  // Radius in meters
            const double theta = 0.0;   // Rotation angle (can be parameterized)

            // Create a list to store yaw angles for each waypoint
            std::vector<double> hexagon_yaws;
            hexagon_yaws.clear();  // Clear previous yaws

            // define the initial heading angle (300 degrees) before generating the hexagon
            const double initial_heading_degrees = 270.0;
            const double angle_decrement = 60.0;  // decrement 60 degrees for each vertex

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

                // decrement the heading angle, ensure in the range of 0-360 degrees
                double heading_degrees =
                    std::fmod(initial_heading_degrees - i * angle_decrement + 360.0, 360.0);

                // store the converted heading angle (degrees)
                hexagon_yaws.push_back(heading_degrees);

                // Add to waypoints array
                hexagon_waypoints.push_back(wp);
            }

            // Add the first point again to complete the loop
            if (!hexagon_waypoints.empty()) {
                hexagon_waypoints.push_back(hexagon_waypoints[0]);
                // Also add the first yaw again
                if (!hexagon_yaws.empty()) {
                    hexagon_yaws.push_back(hexagon_yaws[0]);
                }
                RCLCPP_INFO(this->get_logger(),
                            "Generated %zu hexagon waypoints (including loop closure).",
                            hexagon_waypoints.size());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to generate any hexagon waypoints.");
                survey_action->set_failure();
                return;  // Exit if no waypoints generated
            }

            // Send "LOCK_ON" command to gimbal
            auto cmd_msg = std_msgs::msg::String();
            cmd_msg.data = "LOCK_ON";
            gimbal_command_pub->publish(cmd_msg);
            gimbal_command_pub->publish(cmd_msg);
            gimbal_command_pub->publish(cmd_msg);
            RCLCPP_INFO(this->get_logger(), "Sent LOCK_ON command to gimbal");

            // Clear previous mission before pushing new one
            auto clear_request = std::make_shared<mavros_msgs::srv::WaypointClear::Request>();
            if (!waypoint_clear_client->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_WARN(this->get_logger(),
                            "Waypoint clear service not available, proceeding without clearing.");
            } else {
                auto clear_future = waypoint_clear_client->async_send_request(clear_request);
                // We don't necessarily need to wait for clear to finish before pushing
                clear_future.wait_for(std::chrono::seconds(1));  // Short wait, non-blocking mostly
                RCLCPP_INFO(this->get_logger(), "Requested waypoint clear.");
            }

            // Push the generated waypoints *once*
            push_waypoints(survey_action, hexagon_waypoints, 8,
                           10.0,  // 10s hold time
                           hexagon_yaws, 0.3);

            // Reset the precise detection flag *after* using it to determine center
            precise_casualty_detected = false;
        }
    }
    // publish conditions and actions
    for (bt::Condition* condition : conditions) condition->publish();
    for (bt::Action* action : actions) action->publish();
}

// UPDATE CONDITIONS
// this specific callback subs to behavior_tree_commands
void BehaviorExecutive::bt_commands_callback(behavior_tree_msgs::msg::BehaviorTreeCommands msg) {
    for (size_t i = 0; i < msg.commands.size(); i++) {
        std::string condition_name = msg.commands[i].condition_name;
        int status = msg.commands[i].status;

        for (size_t j = 0; j < conditions.size(); j++) {
            bt::Condition* condition = conditions[j];
            if (condition_name == condition->get_label()) {
                if (status == behavior_tree_msgs::msg::Status::SUCCESS)
                    condition->set(true);
                else if (status == behavior_tree_msgs::msg::Status::FAILURE)
                    condition->set(false);
            }
        }
    }
}

// update based on state msgs
void BehaviorExecutive::state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
    armed_condition->set(msg->armed);
    this->current_mode = msg->mode;
}

void BehaviorExecutive::current_gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    current_latitude = msg->latitude;
    current_longitude = msg->longitude;
}

void BehaviorExecutive::relative_altitude_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    current_relative_altitude = msg->data;
}

void BehaviorExecutive::set_target_altitude_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    target_ascend_altitude = msg->data;
    RCLCPP_INFO(this->get_logger(), "Target altitude set to: %.2f meters", target_ascend_altitude);
}

void BehaviorExecutive::target_waypoint_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    target_waypoint.latitude = msg->latitude;
    target_waypoint.longitude = msg->longitude;
}

void BehaviorExecutive::UAV_Deconflict_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "UAV_Deconflict_callback received");

    // Convert flat array into list of [lat, lon, alt]
    const std::vector<double>& data = msg->data;
    if (data.size() % 3 != 0) {
        RCLCPP_ERROR(this->get_logger(), "Received malformed GPS list");
        return;
    }

    std::vector<std::vector<double>> gps_uav_list;
    for (size_t i = 0; i < data.size(); i += 3) {
        gps_uav_list.push_back({data[i], data[i + 1], data[i + 2]});
    }

    // Set self GPS from current telemetry
    std::vector<double> gps_self = {current_latitude, current_longitude, current_relative_altitude};
    double radius = 5.0;  // Define radius as needed

    // Haversine distance calculation
    auto haversine_distance = [](const std::vector<double>& gps1,
                                 const std::vector<double>& gps2) -> double {
        double lat1 = gps1[0] * M_PI / 180.0;
        double lon1 = gps1[1] * M_PI / 180.0;
        double alt1 = gps1[2];

        double lat2 = gps2[0] * M_PI / 180.0;
        double lon2 = gps2[1] * M_PI / 180.0;
        double alt2 = gps2[2];

        double dlat = lat2 - lat1;
        double dlon = lon2 - lon1;

        double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
                   std::cos(lat1) * std::cos(lat2) * std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
        double c = 2.0 * std::asin(std::sqrt(a));
        const double EARTH_RADIUS = 6371000.0;
        double horizontal_distance = EARTH_RADIUS * c;

        double vertical_distance = std::abs(alt1 - alt2);
        return std::sqrt(horizontal_distance * horizontal_distance +
                         vertical_distance * vertical_distance);
    };

    // Deconfliction logic
    bool conflict = false;
    for (const auto& gps : gps_uav_list) {
        double dist = haversine_distance(gps_self, gps);
        if (dist <= 2.0 * radius) {
            conflict = true;
            break;
        }
    }

    if (conflict) {
        RCLCPP_WARN(this->get_logger(), "Warning: UAV conflict detected. Triggering E-Stop.");

        // Fail all active actions
        for (auto* action : actions) {
            if (action->is_active()) {
                action->set_failure();
            }
        }

        // Activate emergency hold
        auto hold_active_pub =
            this->create_publisher<behavior_tree_msgs::msg::Active>("/behavior/hold_active", 10);

        auto active_msg = behavior_tree_msgs::msg::Active();
        active_msg.active = true;          //
        active_msg.id = ++hold_active_id;  // every time publishing should be differen id

        hold_active_pub->publish(active_msg);

    } else {
        RCLCPP_INFO(this->get_logger(), "Clear: No UAV conflicts detected.");
    }
}

void BehaviorExecutive::set_mode(bt::Action* action,
                                 const rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr& client,
                                 const std::string& mode) {
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->base_mode = 0;
    request->custom_mode = mode;

    if (!client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Set mode service not available");
        action->set_failure();
        return;
    }

    auto result = client->async_send_request(request);
    if (result.wait_for(3s) == std::future_status::timeout) {
        RCLCPP_ERROR(this->get_logger(), "Mode switch timed out");
        action->set_failure();
        return;
    }

    if (!result.get()->mode_sent) {
        RCLCPP_ERROR(this->get_logger(), "Failed to switch to mode: %s", mode.c_str());
        action->set_failure();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Successfully switched to mode: %s", mode.c_str());
    RCLCPP_INFO(this->get_logger(), "Action {%s}Success: ", action->get_label().c_str());
    action->set_success();
}

bool BehaviorExecutive::is_point_in_fence(const std::pair<double, double>& point,
                                          const std::vector<std::pair<double, double>>& fence) {
    // Implementation of point-in-polygon algorithm (ray casting algorithm)
    int num_vertices = fence.size();
    bool inside = false;

    // Test point coordinates
    double testx = point.first;
    double testy = point.second;

    // Loop through all edges of the polygon
    for (int i = 0, j = num_vertices - 1; i < num_vertices; j = i++) {
        // Vertex coordinates
        double vertx_i = fence[i].first;
        double verty_i = fence[i].second;
        double vertx_j = fence[j].first;
        double verty_j = fence[j].second;

        // Check if the ray horizontally from the point crosses this edge
        if (((verty_i > testy) != (verty_j > testy)) &&
            (testx < (vertx_j - vertx_i) * (testy - verty_i) / (verty_j - verty_i) + vertx_i)) {
            // Toggle inside/outside status
            inside = !inside;
        }
    }

    return inside;
}

void BehaviorExecutive::push_waypoints(bt::Action* action,
                                       const std::vector<sensor_msgs::msg::NavSatFix>& waypoints,
                                       double target_altitude, double hold_time,
                                       const std::vector<double>& yaws, double acceptance_radius) {
    auto push_request = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();
    push_request->start_index = 0;  // treat as a new mission, not appending to existing mission

    // Check if yaws are provided and match the waypoints count
    bool use_yaws = !yaws.empty() && yaws.size() == waypoints.size();
    if (!yaws.empty() && yaws.size() != waypoints.size()) {
        RCLCPP_WARN(this->get_logger(),
                    "Yaw angles count (%zu) doesn't match waypoints count (%zu), ignoring yaws",
                    yaws.size(), waypoints.size());
    }

    // Add each waypoint to the request
    for (size_t i = 0; i < waypoints.size(); i++) {
        mavros_msgs::msg::Waypoint wp;
        wp.frame = mavros_msgs::msg::Waypoint::FRAME_GLOBAL_REL_ALT;
        wp.command = 16;           // MAV_CMD_NAV_WAYPOINT
        wp.is_current = (i == 0);  // Only the first waypoint is current
        wp.autocontinue = true;
        wp.param1 = hold_time;                 // Hold time
        wp.param2 = acceptance_radius;         // Acceptance radius - Use the new parameter
        wp.param3 = 0.0;                       // Pass through
        wp.param4 = use_yaws ? yaws[i] : NAN;  // Use the input yaw or NAN if not provided
        wp.x_lat = waypoints[i].latitude;
        wp.y_long = waypoints[i].longitude;
        wp.z_alt = target_altitude;
        RCLCPP_INFO(this->get_logger(), "yaw=%f", wp.param4);

        push_request->waypoints.push_back(wp);
    }

    if (!waypoint_push_client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Waypoint push service not available");
        action->set_failure();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Pushing %zu waypoints", waypoints.size());

    auto push_future = waypoint_push_client->async_send_request(
        push_request, [this, action, waypoints_count = waypoints.size()](
                          rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedFuture future) {
            if (future.valid()) {
                auto result = future.get();
                if (!result->success) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to push waypoints");
                    action->set_failure();
                } else {
                    RCLCPP_INFO(this->get_logger(), "Successfully pushed %zu waypoints",
                                waypoints_count);
                    if (this->current_mode == "AUTO.MISSION") {
                        action->set_success();
                    } else {
                        set_mode(action, set_mode_client, "AUTO.MISSION");
                    }
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Waypoint push future invalid");
                action->set_failure();
            }
        });
}

void BehaviorExecutive::geofence_callback(const mavros_msgs::msg::WaypointList::SharedPtr msg) {
    // Only load geofence points if we haven't loaded them yet
    if (geofence_points.empty()) {
        for (const auto& wp : msg->waypoints) {
            std::pair<double, double> point = {wp.x_lat, wp.y_long};
            geofence_points.push_back(point);
        }

        if (!geofence_points.empty()) {
            RCLCPP_INFO(this->get_logger(), "Received %zu geofence points", geofence_points.size());
        }
    }
}

void BehaviorExecutive::target_gps_list_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (msg->header.frame_id.find("casualty_0") == 0) {
        target_gps.latitude = msg->latitude;
        target_gps.longitude = msg->longitude;
        target_gps.altitude = msg->altitude;

        RCLCPP_INFO(this->get_logger(), "Averaged Casualty_0 GPS: lat=%f, lon=%f",
                    target_gps.latitude, target_gps.longitude);
    } else {
        RCLCPP_DEBUG(this->get_logger(),
                     "Received message on target_gps_list topic with frame_id: %s",
                     msg->header.frame_id.c_str());
    }
}

void BehaviorExecutive::target_gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    // the latest ray-casting estimated GPS

    float confidence = msg->altitude;  // Lower value means higher confidence
    // compare with best confidence seen so far
    if (confidence < min_confidence) {
        min_confidence = confidence;
        casualty_gps_position[0] = msg->latitude;
        casualty_gps_position[1] = msg->longitude;
    }
    RCLCPP_INFO(this->get_logger(),
                "[ESTIMATION RAY CASTING - INFO] Casualty at lat=%f, lon=%f, confidence=%f, "
                "timestamp=%d.%d",
                casualty_gps_position[0], casualty_gps_position[1], min_confidence,
                msg->header.stamp.sec, msg->header.stamp.nanosec);
}

std::vector<sensor_msgs::msg::NavSatFix> BehaviorExecutive::generate_rectangle_waypoints(
    double center_lat, double center_lon, double width_meters, double height_meters,
    double altitude) {
    std::vector<sensor_msgs::msg::NavSatFix> waypoints;

    // convert degrees to meters
    const double meters_per_lat = 111111.0;  // 1 degree latitude is approximately 111111 meters
    const double meters_per_lon =
        111111.0 * std::cos(center_lat * M_PI /
                            180.0);  // longitude conversion needs to consider latitude factor

    // calculate the offset of the four corners of the rectangle
    double half_width_deg = (width_meters / 2.0) / meters_per_lon;
    double half_height_deg = (height_meters / 2.0) / meters_per_lat;

    // create the four corners and the center point, in clockwise order
    sensor_msgs::msg::NavSatFix wp;

    // top left corner
    wp.latitude = center_lat + half_height_deg;
    wp.longitude = center_lon - half_width_deg;
    wp.altitude = altitude;
    waypoints.push_back(wp);

    // top right corner
    wp.latitude = center_lat + half_height_deg;
    wp.longitude = center_lon + half_width_deg;
    waypoints.push_back(wp);

    // bottom right corner
    wp.latitude = center_lat - half_height_deg;
    wp.longitude = center_lon + half_width_deg;
    waypoints.push_back(wp);

    // bottom left corner
    wp.latitude = center_lat - half_height_deg;
    wp.longitude = center_lon - half_width_deg;
    waypoints.push_back(wp);

    // back to the first point to complete the loop
    wp.latitude = center_lat + half_height_deg;
    wp.longitude = center_lon - half_width_deg;
    waypoints.push_back(wp);

    return waypoints;
}

// 添加新函数用于生成圆形航点
std::vector<sensor_msgs::msg::NavSatFix> BehaviorExecutive::generate_circle_waypoints(
    double center_lat, double center_lon, double radius_meters, int num_points, double altitude) {
    std::vector<sensor_msgs::msg::NavSatFix> waypoints;
    const int target_points = num_points;  // 固定生成10个航点

    // 经纬度与米的转换
    const double meters_per_lat = 111111.0;  // 1度纬度约等于111111米
    const double meters_per_lon =
        111111.0 * std::cos(center_lat * M_PI / 180.0);  // 经度转换需考虑纬度因素

    // 首先添加中心点
    sensor_msgs::msg::NavSatFix center_wp;
    center_wp.latitude = center_lat;
    center_wp.longitude = center_lon;
    center_wp.altitude = altitude;
    waypoints.push_back(center_wp);

    // 生成剩余9个点均匀分布在圆形区域内
    const int remaining_points = target_points - 1;

    // 使用Sunflower seed arrangement算法生成均匀分布点
    const double golden_ratio = (1.0 + std::sqrt(5.0)) / 2.0;

    for (int i = 0; i < remaining_points; i++) {
        // 计算当前点的半径和角度
        double dist = std::sqrt(static_cast<double>(i) / remaining_points);
        double radius = radius_meters * dist;

        double theta = 2.0 * M_PI * i / golden_ratio;

        double dx = radius * std::cos(theta);
        double dy = radius * std::sin(theta);

        double dlon = dx / meters_per_lon;
        double dlat = dy / meters_per_lat;

        sensor_msgs::msg::NavSatFix wp;
        wp.latitude = center_lat + dlat;
        wp.longitude = center_lon + dlon;
        wp.altitude = altitude;
        waypoints.push_back(wp);
    }

    return waypoints;
}

void BehaviorExecutive::precise_target_gps_callback(
    const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    // Only process if search action is active and running
    if (!search_action->is_active() || !search_action->is_running()) {
        return;
    }

    RCLCPP_INFO(this->get_logger(),
                "[***CRITICAL*** SEARCH PRECISE FOUND] Precise target detected!");
    //
    precise_casualty_gps_position[0] = current_latitude;   // msg->latitude;
    precise_casualty_gps_position[1] = current_longitude;  // msg->longitude;
    precise_casualty_detected = true;

    RCLCPP_INFO(this->get_logger(),
                "Stored casualty position at current drone location: lat=%f, lon=%f",
                precise_casualty_gps_position[0], precise_casualty_gps_position[1]);
}

// Add the point_to_line_segment_distance function
double BehaviorExecutive::point_to_line_segment_distance(
    const std::pair<double, double>& point, const std::pair<double, double>& line_start,
    const std::pair<double, double>& line_end) {
    // Convert to x,y notation for clarity
    double x = point.first;
    double y = point.second;
    double x1 = line_start.first;
    double y1 = line_start.second;
    double x2 = line_end.first;
    double y2 = line_end.second;

    // Calculate squared length of line segment
    double line_length_squared = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

    // If line segment is actually a point, return distance to that point
    if (line_length_squared == 0.0) {
        return std::sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
    }

    // Calculate projection of point onto line segment
    double t = std::max(
        0.0, std::min(1.0, ((x - x1) * (x2 - x1) + (y - y1) * (y2 - y1)) / line_length_squared));

    // Calculate closest point on line segment
    double closest_x = x1 + t * (x2 - x1);
    double closest_y = y1 + t * (y2 - y1);

    // Return distance to closest point
    return std::sqrt((x - closest_x) * (x - closest_x) + (y - closest_y) * (y - closest_y));
}

// Helper function to calculate the cross product of three points
double BehaviorExecutive::cross_product(const std::pair<double, double>& a,
                                        const std::pair<double, double>& b,
                                        const std::pair<double, double>& c) {
    return (b.first - a.first) * (c.second - a.second) -
           (b.second - a.second) * (c.first - a.first);
}

// Function to compute the convex hull using the Graham scan algorithm
std::vector<std::pair<double, double>> BehaviorExecutive::convex_hull(
    std::vector<std::pair<double, double>>& points) {
    int n = points.size();
    if (n <= 2) return points;

    // 1. Find the point with the lowest y-coordinate (and leftmost in case of a tie)
    auto lowest_y_it = std::min_element(
        points.begin(), points.end(),
        [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
            if (a.second != b.second) {
                return a.second < b.second;
            }
            return a.first < b.first;
        });
    std::iter_swap(points.begin(), lowest_y_it);
    std::pair<double, double> pivot = points[0];

    // 2. Sort the remaining points based on the polar angle they make with the pivot
    std::sort(
        points.begin() + 1, points.end(),
        [&](const std::pair<double, double>& a, const std::pair<double, double>& b) {
            double orientation = cross_product(pivot, a, b);
            if (orientation == 0) {
                // If collinear, sort by distance from pivot
                return std::pow(a.first - pivot.first, 2) + std::pow(a.second - pivot.second, 2) <
                       std::pow(b.first - pivot.first, 2) + std::pow(b.second - pivot.second, 2);
            }
            return orientation > 0;  // Counter-clockwise order
        });

    // 3. Build the convex hull
    std::vector<std::pair<double, double>> hull;
    hull.push_back(points[0]);
    hull.push_back(points[1]);

    for (int i = 2; i < n; ++i) {
        while (hull.size() >= 2 &&
               cross_product(hull[hull.size() - 2], hull.back(), points[i]) <= 0) {
            hull.pop_back();
        }
        hull.push_back(points[i]);
    }

    return hull;
}

// Add the is_point_safe_distance function if it doesn't exist in the .cpp file
bool BehaviorExecutive::is_point_safe_distance(const std::pair<double, double>& point,
                                               const std::vector<std::pair<double, double>>& fence,
                                               double safe_dist_lat, double safe_dist_lon) {
    // If the fence has fewer than 2 points, we can't check distances
    if (fence.size() < 2) {
        return true;  // Assume it's safe as there's no real fence
    }

    double min_distance = std::numeric_limits<double>::max();

    // Check distance to each fence segment
    for (size_t i = 0; i < fence.size(); i++) {
        size_t j = (i + 1) % fence.size();  // Get next point, wrapping around to the start

        double dist = point_to_line_segment_distance(point, fence[i], fence[j]);
        min_distance = std::min(min_distance, dist);
    }

    // Convert min_distance from degrees to approximate meters
    // This is a rough approximation - we'd need to take into account local latitude for
    // accuracy
    const double meters_per_lat_deg = 111111.0;  // Use standardized constant
    const double meters_per_lon_deg =
        111111.0 *
        std::cos(point.first * M_PI / 180.0);  // Approx longitude conversion at point's latitude

    // Calculate approximate distance in meters using Pythagoras on degree differences,
    // scaling each axis appropriately. This is better than just using latitude scale.
    // Note: This still assumes a flat Earth locally.
    // double min_distance_meters = min_distance * meters_per_lat_deg; // Old simplified conversion

    // We already calculated the minimum distance in degrees (`min_distance`)
    // Let's use the standardized conversion factors. Since `point_to_line_segment_distance` returns
    // distance in degrees, and the safe distance is provided in degrees (safe_dist_lat,
    // safe_dist_lon), we should ideally compare distances in degrees directly if the safe distance
    // parameters accurately represent the required meters converted to degrees *at the relevant
    // location*. However, the parameters are named `safe_dist_lat`, `safe_dist_lon`, implying they
    // might already be scaled. Let's stick to comparing the calculated degree distance to the input
    // degree thresholds for now, assuming the caller has correctly calculated these thresholds. The
    // conversion to meters here was perhaps just for logging/debugging previously. If we *must*
    // compare in meters, the conversion needs care. For now, let's assume the comparison should
    // happen in degrees.

    // Check if the point is closer than the safe distance thresholds (in degrees)
    // This comparison logic might need review based on how safe_dist_lat/lon are derived.
    // Assuming min_distance is a scalar distance in degrees, comparing it directly to
    // safe_dist_lat and safe_dist_lon separately might not be correct.
    // Let's refine the check: Is the point closer than the minimum required safe distance (in
    // degrees)? We need a single safe distance threshold in degrees for comparison. Let's assume
    // safe_dist_lat is the primary one for now. This part needs clarification on the intent of
    // safe_dist_lat/lon. Reverting to a simple check against safe_dist_lat for now.

    return min_distance >= safe_dist_lat;  // Compare distance in degrees
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = std::make_shared<BehaviorExecutive>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

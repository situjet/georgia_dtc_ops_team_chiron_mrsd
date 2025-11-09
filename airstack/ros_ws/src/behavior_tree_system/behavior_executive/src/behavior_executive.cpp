#include <rmw/qos_profiles.h>  // For rmw_qos_profile_services_default

#include <airstack_msgs/srv/trajectory_mode.hpp>
#include <algorithm>  // For std::min, std::max
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

// Mission planning utilities (must be before behavior_executive.hpp)
#include <mission_planning/geometry_utils.hpp>
#include <mission_planning/waypoint_generator.hpp>

// Action handlers (must be before behavior_executive.hpp)
#include <action_handlers/geofence_mapping_handler.hpp>
#include <action_handlers/navigate_to_waypoint_handler.hpp>
#include <action_handlers/search_handler.hpp>
#include <action_handlers/survey_handler.hpp>

// Now include behavior_executive and mavros_adapter
#include <behavior_executive.hpp>
#include <mavros_adapter.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>  // For std::vector

// #include "vision_msgs/msg/detection2_d_array.hpp"
using namespace std::chrono_literals;

BehaviorExecutive::BehaviorExecutive(std::shared_ptr<IMavrosAdapter> mavros_adapter) 
    : Node("behavior_executive") {
    // 如果没有提供 MAVROS 适配器，则创建默认实现
    if (!mavros_adapter) {
        mavros_adapter_ = std::make_shared<MavrosAdapter>(this);
    } else {
        mavros_adapter_ = mavros_adapter;
    }

    // 初始化转换记录器
    transition_logger_ = std::make_unique<TransitionLogger>(
        this,
        "",     // 使用默认目录
        true,   // 输出到控制台
        true    // 输出到文件
    );

    // 初始化动作处理器（新架构）
    geometry_utils_ = std::make_shared<GeometryUtils>();
    waypoint_generator_ = std::make_shared<WaypointGenerator>(geometry_utils_);
    geofence_handler_ = std::make_unique<GeofenceMappingHandler>(
        mavros_adapter_,
        waypoint_generator_,
        geometry_utils_,
        this->get_logger()
    );
    navigate_to_waypoint_handler_ = std::make_unique<NavigateToWaypointHandler>(
        mavros_adapter_,
        this->get_logger()
    );
    search_handler_ = std::make_unique<SearchHandler>(
        mavros_adapter_,
        this->get_logger()
    );
    survey_handler_ = std::make_unique<SurveyHandler>(
        mavros_adapter_,
        this->get_logger()
    );

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

    // 11. Navigate to Waypoint Behavior
    navigate_to_waypoint_commanded_condition = 
        new bt::Condition("Navigate to Waypoint Commanded", this);
    navigate_to_waypoint_action = new bt::Action("Navigate to Waypoint", this);

    conditions.push_back(auto_takeoff_commanded_condition);
    conditions.push_back(armed_condition);
    conditions.push_back(survey_commanded_condition);

    conditions.push_back(arm_commanded_condition);
    conditions.push_back(geofence_mapping_commanded_condition);
    conditions.push_back(disarm_commanded_condition);
    conditions.push_back(e_stop_commanded_condition);
    conditions.push_back(auto_land_commanded_condition);
    conditions.push_back(search_commanded_condition);
    conditions.push_back(navigate_to_waypoint_commanded_condition);
    // actions

    actions.push_back(arm_action);
    actions.push_back(auto_takeoff_action);
    actions.push_back(hold_action);
    actions.push_back(disarm_action);
    actions.push_back(auto_land_action);

    actions.push_back(geofence_mapping_action);
    actions.push_back(search_action);
    actions.push_back(survey_action);
    actions.push_back(navigate_to_waypoint_action);

    // Subscribers Callback
    behavior_tree_commands_sub =
        this->create_subscription<behavior_tree_msgs::msg::BehaviorTreeCommands>(
            "behavior_tree_commands", 1,
            std::bind(&BehaviorExecutive::bt_commands_callback, this, std::placeholders::_1));
    state_sub = this->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", rclcpp::QoS(10).best_effort(),
        std::bind(&BehaviorExecutive::state_callback, this, std::placeholders::_1));
    current_gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/mavros/global_position/global", rclcpp::QoS(10).best_effort(),
        std::bind(&BehaviorExecutive::current_gps_callback, this, std::placeholders::_1));
    relative_altitude_sub = this->create_subscription<std_msgs::msg::Float64>(
        "/mavros/global_position/rel_alt", rclcpp::QoS(10).best_effort(),
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
    
    selected_waypoint_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/selected_waypoint", 10,
        std::bind(&BehaviorExecutive::selected_waypoint_callback, this, std::placeholders::_1));

    // Service Clients (保留未使用的客户端，未来重构)
    auto service_callback_group =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    takeoff_command_client = this->create_client<mavros_msgs::srv::CommandTOL>(
        "mavros/cmd/takeoff", rmw_qos_profile_services_default, service_callback_group);
    land_command_client = this->create_client<mavros_msgs::srv::CommandTOL>(
        "mavros/cmd/land", rmw_qos_profile_services_default, service_callback_group);

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

    // 初始化 Navigate to Waypoint 变量
    selected_waypoint_lat_ = 0.0;
    selected_waypoint_lon_ = 0.0;
    selected_waypoint_alt_ = 10.0;  // 默认高度

    // // Added for the new drone_pose_callback
    // current_heading_radians = 0.0;  // Heading in radians in ENU frame
    // current_heading_degrees = 0.0;  // Heading in degrees [0, 360)

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
            // 记录action激活
            log_action_transition(auto_takeoff_action, "ACTIVATED");
            
            // Set terrain altitude to current relative altitude
            terrain_altitude = current_relative_altitude;
            RCLCPP_INFO(this->get_logger(), "Setting terrain altitude to: %f meters",
                        terrain_altitude);
            // Adds offset to target_ascend_altitude
            target_ascend_altitude = terrain_altitude + target_ascend_altitude;
            RCLCPP_INFO(this->get_logger(), "Target Flight Altitude Updated: %f meters",
                        target_ascend_altitude);

            // Set to takeoff mode
            set_mode(auto_takeoff_action, "AUTO.TAKEOFF");
        }
    }

    // 2. Arm
    if (arm_action->is_active()) {
        arm_action->set_running();
        if (arm_action->active_has_changed()) {
            // 记录action激活
            log_action_transition(arm_action, "ACTIVATED");
            
            mavros_adapter_->arm_vehicle(
                [this](bool success) {
                    if (success) {
                        arm_action->set_success();
                        log_action_transition(arm_action, "SUCCESS");
                    } else {
                        arm_action->set_failure();
                        log_action_transition(arm_action, "FAILURE", "Arming command failed");
                    }
                },
                arm_action->get_label());
        }
    }

    // 3. Disarm
    if (disarm_action->is_active()) {
        disarm_action->set_running();
        if (disarm_action->active_has_changed()) {
            // 记录action激活
            log_action_transition(disarm_action, "ACTIVATED");
            
            mavros_adapter_->disarm_vehicle(
                [this](bool success) {
                    if (success) {
                        disarm_action->set_success();
                        log_action_transition(disarm_action, "SUCCESS");
                    } else {
                        disarm_action->set_failure();
                        log_action_transition(disarm_action, "FAILURE", "Disarming command failed");
                    }
                },
                disarm_action->get_label());
        }
    }

    // 4. Auto Land
    if (auto_land_action->is_active()) {
        auto_land_action->set_running();
        if (auto_land_action->active_has_changed()) {
            // 记录action激活
            log_action_transition(auto_land_action, "ACTIVATED");
            
            if (this->current_mode != "AUTO.LAND") {
                set_mode(auto_land_action, "AUTO.LAND");
            }
        }
    }

    // 5. E-Stop
    if (hold_action->is_active()) {
        hold_action->set_running();
        if (hold_action->active_has_changed()) {
            // 记录action激活
            log_action_transition(hold_action, "ACTIVATED");
            
            if (this->current_mode == "AUTO.LOITER") {
                RCLCPP_INFO(this->get_logger(), "Already in AUTO.LOITER mode");
                hold_action->set_success();
                log_action_transition(hold_action, "SUCCESS", "", "Already in LOITER mode");
                return;
            }

            set_mode(hold_action, "AUTO.LOITER");
            // print hold_action status
            // RCLCPP_INFO(this->get_logger(), "Hold action status: %d", hold_action->is_success());
        }
    }

    // 6. Geofence Mapping (重构后使用处理器模式)
    if (geofence_mapping_action->is_active()) {
        geofence_mapping_action->set_running();
        if (geofence_mapping_action->active_has_changed()) {
            // 记录action激活
            log_action_transition(geofence_mapping_action, "ACTIVATED");
            
            // 构建上下文
            IActionHandler::Context ctx{
                current_latitude,
                current_longitude,
                target_ascend_altitude,
                current_mode
            };
            
            // 安全检查
            if (!geofence_handler_->check_safety(ctx)) {
                geofence_mapping_action->set_failure();
                log_action_transition(geofence_mapping_action, "FAILURE", "Safety check failed");
                return;
            }

            // 委托给处理器执行
            geofence_handler_->on_activated(geofence_mapping_action, ctx);
        }
    }

    // 7. Search (重构后使用处理器模式)
    if (search_action->is_active()) {
        search_action->set_running();

        if (search_action->active_has_changed()) {
            log_action_transition(search_action, "ACTIVATED");
            
            // 设置目标位置
            if (precise_casualty_detected) {
                search_handler_->set_target_position(precise_casualty_gps_position, true);
            } else {
                search_handler_->set_target_position(casualty_gps_position, false);
            }

            // 构建上下文
            IActionHandler::Context ctx{
                current_latitude,
                current_longitude,
                target_ascend_altitude,
                current_mode
            };

            // 安全检查
            if (!search_handler_->check_safety(ctx)) {
                search_action->set_failure();
                log_action_transition(search_action, "FAILURE", "Safety check failed");
                return;
            }

            // 委托给处理器执行
            search_handler_->on_activated(search_action, ctx);
        }
    }

    // 8. Survey (重构后使用处理器模式)
    if (survey_action->is_active()) {
        survey_action->set_running();

        if (survey_action->active_has_changed()) {
            log_action_transition(survey_action, "ACTIVATED");
            
            // 设置目标位置
            if (precise_casualty_detected) {
                survey_handler_->set_target_position(precise_casualty_gps_position, true);
            }
            // else: 使用默认位置（在 handler 中已定义）

            // 构建上下文
            IActionHandler::Context ctx{
                current_latitude,
                current_longitude,
                target_ascend_altitude,
                current_mode
            };

            // 安全检查
            if (!survey_handler_->check_safety(ctx)) {
                survey_action->set_failure();
                log_action_transition(survey_action, "FAILURE", "Safety check failed");
                return;
            }

            // 委托给处理器执行
            survey_handler_->on_activated(survey_action, ctx);

            // 重置精确检测标志（已在 handler 中处理）
            precise_casualty_detected = false;
        }
    }

    // 9. Navigate to Waypoint
    if (navigate_to_waypoint_action->is_active()) {
        navigate_to_waypoint_action->set_running();
        if (navigate_to_waypoint_action->active_has_changed()) {
            log_action_transition(navigate_to_waypoint_action, "ACTIVATED");

            // 构建上下文
            IActionHandler::Context ctx{
                current_latitude,
                current_longitude,
                selected_waypoint_alt_,  // 使用选中航点的高度
                current_mode
            };

            // 安全检查
            if (!navigate_to_waypoint_handler_->check_safety(ctx)) {
                navigate_to_waypoint_action->set_failure();
                log_action_transition(navigate_to_waypoint_action, "FAILURE", 
                                    "Safety check failed: Invalid waypoint");
                return;
            }

            // 委托给处理器执行
            navigate_to_waypoint_handler_->on_activated(navigate_to_waypoint_action, ctx);
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
                // 记录条件变化前的状态
                bool old_value = condition->get();
                bool new_value = false;
                
                if (status == behavior_tree_msgs::msg::Status::SUCCESS) {
                    condition->set(true);
                    new_value = true;
                } else if (status == behavior_tree_msgs::msg::Status::FAILURE) {
                    condition->set(false);
                    new_value = false;
                }
                
                // 如果值发生了变化，记录到日志
                if (old_value != new_value && transition_logger_) {
                    transition_logger_->log_condition_change(
                        condition_name,
                        old_value,
                        new_value);
                }
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

void BehaviorExecutive::set_mode(bt::Action* action, const std::string& mode) {
    mavros_adapter_->set_mode(
        mode,
        [this, action](bool success) {
            if (success) {
                action->set_success();
                log_action_transition(action, "SUCCESS");
            } else {
                action->set_failure();
                log_action_transition(action, "FAILURE", "Set mode command failed");
            }
        },
        action->get_label());
}

void BehaviorExecutive::push_waypoints(bt::Action* action,
                                       const std::vector<sensor_msgs::msg::NavSatFix>& waypoints,
                                       double target_altitude, double hold_time,
                                       const std::vector<double>& yaws, double acceptance_radius) {
    mavros_adapter_->push_waypoints(
        waypoints,
        target_altitude,
        hold_time,
        yaws,
        acceptance_radius,
        [this, action](bool success) {
            if (success) {
                // 如果当前模式不是 AUTO.MISSION，需要切换模式
                if (this->current_mode != "AUTO.MISSION") {
                    set_mode(action, "AUTO.MISSION");
                } else {
                    action->set_success();
                }
            } else {
                action->set_failure();
            }
        },
        action->get_label(),
        this->current_mode);
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
            
            // 将围栏点传递给处理器
            geofence_handler_->set_geofence_points(geofence_points);
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

void BehaviorExecutive::selected_waypoint_callback(
    const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    
    selected_waypoint_lat_ = msg->latitude;
    selected_waypoint_lon_ = msg->longitude;
    selected_waypoint_alt_ = msg->altitude > 0 ? msg->altitude : target_ascend_altitude;

    // 将坐标传递给 handler
    navigate_to_waypoint_handler_->set_target_waypoint(
        selected_waypoint_lat_, 
        selected_waypoint_lon_, 
        selected_waypoint_alt_
    );
    
    RCLCPP_INFO(this->get_logger(),
                "[Navigate to Waypoint] Received waypoint: lat=%.6f, lon=%.6f, alt=%.1f",
                selected_waypoint_lat_, selected_waypoint_lon_, selected_waypoint_alt_);
}

std::map<std::string, bool> BehaviorExecutive::get_related_conditions(bt::Action* action) {
    std::map<std::string, bool> related_conditions;
    
    // 根据action类型返回相关的conditions
    std::string action_label = action->get_label();
    
    if (action_label == "Request AutoTakeoff") {
        related_conditions["Armed"] = armed_condition->get();
        related_conditions["Auto Takeoff Commanded"] = auto_takeoff_commanded_condition->get();
    } else if (action_label == "Arm") {
        related_conditions["Armed"] = armed_condition->get();
        related_conditions["Arm Commanded"] = arm_commanded_condition->get();
    } else if (action_label == "Disarm") {
        related_conditions["Armed"] = armed_condition->get();
        related_conditions["Disarm Commanded"] = disarm_commanded_condition->get();
    } else if (action_label == "AutoLand") {
        related_conditions["AutoLand Commanded"] = auto_land_commanded_condition->get();
    } else if (action_label == "Hold") {
        related_conditions["EStop Commanded"] = e_stop_commanded_condition->get();
    } else if (action_label == "Geofence Mapping") {
        related_conditions["Geofence Mapping Commanded"] = geofence_mapping_commanded_condition->get();
    } else if (action_label == "Search") {
        related_conditions["Search Commanded"] = search_commanded_condition->get();
    } else if (action_label == "Survey") {
        related_conditions["Survey Commanded"] = survey_commanded_condition->get();
    } else if (action_label == "Navigate to Waypoint") {
        related_conditions["Navigate to Waypoint Commanded"] = navigate_to_waypoint_commanded_condition->get();
    }
    
    return related_conditions;
}

void BehaviorExecutive::log_action_transition(
    bt::Action* action,
    const std::string& status,
    const std::string& failure_reason,
    const std::string& additional_info) {
    
    if (!transition_logger_) {
        return;  // Logger not initialized
    }
    
    std::map<std::string, bool> conditions = get_related_conditions(action);
    
    transition_logger_->log_action_transition(
        action->get_label(),
        status,
        conditions,
        failure_reason,
        additional_info
    );
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

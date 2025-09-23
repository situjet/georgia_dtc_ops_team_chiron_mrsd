// behavior_governor.cpp
// C++ node for core vehicle behaviors, exposes topics/services for Python extension
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/waypoint_push.hpp>
#include <mavros_msgs/srv/waypoint_clear.hpp>
#include <mavros_msgs/msg/waypoint.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <rmw/qos_profiles.h>
#include "behavior_governor/srv/push_waypoints.hpp"
#include <chrono>
#include <algorithm>
#include <vector>
#include <limits>
#include <cmath>

using std::placeholders::_1;
using namespace std::chrono_literals;


class BehaviorGovernor : public rclcpp::Node {
public:
  BehaviorGovernor() : Node("behavior_governor") {
    // Subscribers for high-level behavior commands (from Python or other nodes)
    behavior_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
      "behavior_governor/command", 10,
      std::bind(&BehaviorGovernor::on_behavior_command, this, _1));

    // Publisher for status
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "behavior_governor/status", 10);
    
    // Publisher for current state (new)
    state_pub_ = this->create_publisher<mavros_msgs::msg::State>(
      "behavior_governor/state", 10);

    // Example: publisher for waypoint
    waypoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "mavros/setpoint_position/local", 10);

    // State subscriber (getting vehicle state)
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "mavros/state", 10,
      std::bind(&BehaviorGovernor::on_state, this, _1));

    // Altitude subscriber for airborne detection
    altitude_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "mavros/global_position/rel_alt", rclcpp::QoS(10).best_effort(),
      std::bind(&BehaviorGovernor::on_altitude, this, _1));

    // Initialize MAVROS service clients
    // Using a separate callback group for service clients to prevent deadlock
    // when calling services from within subscription callbacks.
    // MutuallyExclusive ensures thread safety for service calls.
    service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    arm_disarm_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
      "mavros/cmd/arming", rmw_qos_profile_services_default, service_callback_group_);
    
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
      "mavros/set_mode", rmw_qos_profile_services_default, service_callback_group_);
    
    waypoint_push_client_ = this->create_client<mavros_msgs::srv::WaypointPush>(
      "mavros/mission/push", rmw_qos_profile_services_default, service_callback_group_);
    
    waypoint_clear_client_ = this->create_client<mavros_msgs::srv::WaypointClear>(
      "mavros/mission/clear", rmw_qos_profile_services_default, service_callback_group_);

    // Create waypoint push service
    waypoint_service_ = this->create_service<behavior_governor::srv::PushWaypoints>(
      "behavior_governor/push_waypoints",
      std::bind(&BehaviorGovernor::handle_waypoint_service, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize state - will be updated by MAVROS callbacks
    armed_ = false;
    mode_ = "";
    airborne_ = false;
    current_altitude_ = 0.0;
    state_initialized_ = false;
    
    // Safety thresholds
    airborne_threshold_ = 0.2;  // meters - consider airborne if above this altitude
    
    RCLCPP_INFO(this->get_logger(), "BehaviorGovernor initialized as Executive+Safety layer");
    RCLCPP_INFO(this->get_logger(), "Provides atomic behaviors only - complex behaviors should be in higher-level nodes");
    RCLCPP_INFO(this->get_logger(), "Waiting for MAVROS state sync...");
  }

private:
  // State variables
  bool armed_;
  std::string mode_;
  bool airborne_;
  double current_altitude_;
  bool state_initialized_;
  
  // Safety thresholds
  double airborne_threshold_;

  void on_state(const mavros_msgs::msg::State::SharedPtr msg) {
    // Track mode changes
    std::string previous_mode = mode_;
    bool previous_armed = armed_;
    
    armed_ = msg->armed;
    mode_ = msg->mode;
    
    // Publish current state for external nodes
    mavros_msgs::msg::State state_msg;
    state_msg.armed = armed_;
    state_msg.mode = mode_;
    state_msg.connected = msg->connected;
    state_msg.guided = msg->guided;
    state_msg.manual_input = msg->manual_input;
    state_msg.system_status = msg->system_status;
    state_pub_->publish(state_msg);
    
    // Log important state changes
    if (state_initialized_) {
      if (previous_mode != mode_) {
        RCLCPP_INFO(this->get_logger(), "Flight mode changed: %s -> %s", 
                    previous_mode.c_str(), mode_.c_str());
        validate_mode_transition(previous_mode, mode_);
      }
      if (previous_armed != armed_) {
        RCLCPP_INFO(this->get_logger(), "Armed state changed: %s -> %s", 
                    previous_armed ? "ARMED" : "DISARMED", 
                    armed_ ? "ARMED" : "DISARMED");
      }
    } else {
      state_initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "MAVROS state synchronized: armed=%s, mode=%s", 
                  armed_ ? "true" : "false", mode_.c_str());
    }
  }

  void on_altitude(const std_msgs::msg::Float64::SharedPtr msg) {
    current_altitude_ = msg->data;
    
    // Update airborne status based on altitude threshold
    bool was_airborne = airborne_;
    airborne_ = (current_altitude_ > airborne_threshold_);
    
    // Log airborne state changes
    if (was_airborne != airborne_) {
      RCLCPP_INFO(this->get_logger(), "Airborne state changed: %s (altitude: %.2f m)", 
                  airborne_ ? "AIRBORNE" : "GROUND", current_altitude_);
    }
  }

  void on_behavior_command(const std_msgs::msg::String::SharedPtr msg) {
    // Check if MAVROS state is initialized
    if (!state_initialized_) {
      publish_status("Command rejected: MAVROS state not yet synchronized");
      return;
    }
    
    // Parse command and validate conditions before state switch
    if (msg->data == "arm") {
      handle_arm_command();
    } else if (msg->data == "disarm") {
      handle_disarm_command();
    } else if (msg->data == "takeoff") {
      handle_mode_command("AUTO.TAKEOFF");
    } else if (msg->data == "land") {
      handle_mode_command("AUTO.LAND");
    } else if (msg->data == "hold" || msg->data == "loiter") {
      handle_mode_command("AUTO.LOITER");
    } else if (msg->data == "rtl") {
      handle_mode_command("AUTO.RTL");
    } else if (msg->data == "mission") {
      handle_mode_command("AUTO.MISSION");
    } else if (msg->data == "clear_waypoints") {
      handle_clear_waypoints();
    } else if (msg->data == "goto_waypoint") {
      if (!armed_ || !airborne_) {
        publish_status("Goto waypoint rejected: must be armed and airborne");
        return;
      }
      publish_status("Going to waypoint");
      // ... (publish waypoint)
    } else {
      publish_status("Unknown command: " + msg->data);
    }
  }

  void publish_status(const std::string &status) {
    auto msg = std_msgs::msg::String();
    msg.data = status;
    status_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Status: %s", status.c_str());
  }

  // PX4 Mode validation and management
  bool is_valid_px4_mode(const std::string& mode) {
    // List of supported PX4 modes based on behavior_executive
    static const std::vector<std::string> valid_modes = {
      "MANUAL", "ACRO", "ALTCTL", "POSCTL", "OFFBOARD",
      "AUTO.READY", "AUTO.TAKEOFF", "AUTO.LOITER", "AUTO.MISSION", 
      "AUTO.RTL", "AUTO.LAND", "AUTO.RTGS", "AUTO.FOLLOW_TARGET"
    };
    
    return std::find(valid_modes.begin(), valid_modes.end(), mode) != valid_modes.end();
  }

  void validate_mode_transition(const std::string& from_mode, const std::string& to_mode) {
    if (!is_valid_px4_mode(to_mode)) {
      RCLCPP_WARN(this->get_logger(), "Unknown PX4 mode detected: %s", to_mode.c_str());
      return;
    }

    // Log critical mode transitions
    if (to_mode == "AUTO.LAND") {
      RCLCPP_WARN(this->get_logger(), "CRITICAL: Vehicle entering landing mode");
    } else if (to_mode == "AUTO.RTL") {
      RCLCPP_WARN(this->get_logger(), "CRITICAL: Vehicle entering return-to-launch mode");
    } else if (to_mode == "OFFBOARD") {
      RCLCPP_INFO(this->get_logger(), "Vehicle entering offboard control mode");
    }
  }

  bool can_switch_to_mode(const std::string& target_mode) {
    if (!state_initialized_) {
      publish_status("Mode switch rejected: MAVROS state not synchronized");
      return false;
    }

    if (!is_valid_px4_mode(target_mode)) {
      publish_status("Mode switch rejected: Invalid PX4 mode: " + target_mode);
      return false;
    }

    // Mode-specific pre-conditions
    if (target_mode == "AUTO.TAKEOFF") {
      if (!armed_) {
        publish_status("AUTO.TAKEOFF rejected: vehicle not armed");
        return false;
      }
      if (airborne_) {
        publish_status("AUTO.TAKEOFF rejected: already airborne");
        return false;
      }
    } else if (target_mode == "AUTO.LAND") {
      if (!airborne_) {
        publish_status("AUTO.LAND rejected: vehicle not airborne");
        return false;
      }
    } else if (target_mode == "AUTO.MISSION") {
      if (!armed_) {
        publish_status("AUTO.MISSION rejected: vehicle not armed");
        return false;
      }
      // Note: Could add waypoint validation here
    }

    return true;
  }

  // Mode switching command handler
  void handle_mode_command(const std::string& target_mode) {
    if (!can_switch_to_mode(target_mode)) {
      return;
    }

    // Check if already in target mode
    if (mode_ == target_mode) {
      publish_status("Already in mode: " + target_mode);
      return;
    }

    publish_status("Switching to mode: " + target_mode);

    // Check if service is available
    if (!set_mode_client_->wait_for_service(std::chrono::seconds(2))) {
      publish_status("Mode switch failed: MAVROS set_mode service not available");
      return;
    }

    // Create service request
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->base_mode = 0;
    request->custom_mode = target_mode;

    // Call service asynchronously
    auto future = set_mode_client_->async_send_request(request);
    
    // Wait for response with timeout
    if (future.wait_for(std::chrono::seconds(3)) == std::future_status::ready) {
      auto response = future.get();
      if (response->mode_sent) {
        publish_status("Mode switch command sent successfully: " + target_mode);
        // Note: actual mode change will be updated via state callback
      } else {
        publish_status("Mode switch failed: PX4 rejected mode " + target_mode);
      }
    } else {
      publish_status("Mode switch timed out for mode: " + target_mode);
    }
  }

  // Waypoint management functions
  // Synchronous clear helper used before pushing a fresh plan
  bool clear_waypoints_sync(std::chrono::milliseconds timeout = std::chrono::milliseconds(3000)) {
    publish_status("Clearing existing mission before push...");

    if (!waypoint_clear_client_->wait_for_service(std::chrono::seconds(2))) {
      publish_status("Clear waypoints failed: MAVROS waypoint clear service not available");
      return false;
    }

    auto req = std::make_shared<mavros_msgs::srv::WaypointClear::Request>();
    auto fut = waypoint_clear_client_->async_send_request(req);

    if (fut.wait_for(timeout) == std::future_status::ready) {
      auto resp = fut.get();
      if (resp->success) {
        publish_status("Existing mission cleared");
        return true;
      } else {
        publish_status("Clear waypoints failed: PX4 rejected clear request");
        return false;
      }
    } else {
      publish_status("Clear waypoints timed out");
      return false;
    }
  }

  void handle_clear_waypoints() {
    if (!state_initialized_) {
      publish_status("Clear waypoints rejected: MAVROS state not synchronized");
      return;
    }

    publish_status("Clearing waypoints...");

    // Check if service is available
    if (!waypoint_clear_client_->wait_for_service(std::chrono::seconds(2))) {
      publish_status("Clear waypoints failed: MAVROS waypoint clear service not available");
      return;
    }

    // Create service request
    auto request = std::make_shared<mavros_msgs::srv::WaypointClear::Request>();

    // Call service asynchronously
    auto future = waypoint_clear_client_->async_send_request(request);
    
    // Wait for response with timeout
    if (future.wait_for(std::chrono::seconds(3)) == std::future_status::ready) {
      auto response = future.get();
      if (response->success) {
        publish_status("Waypoints cleared successfully");
      } else {
        publish_status("Clear waypoints failed: PX4 rejected clear request");
      }
    } else {
      publish_status("Clear waypoints timed out");
    }
  }

  bool can_push_waypoints(const std::vector<sensor_msgs::msg::NavSatFix>& waypoints) {
    if (!state_initialized_) {
      publish_status("Push waypoints rejected: MAVROS state not synchronized");
      return false;
    }

    if (waypoints.empty()) {
      publish_status("Push waypoints rejected: empty waypoint list");
      return false;
    }

    // Note: pushing missions while disarmed is allowed; execution requires arming + AUTO.MISSION
    if (!armed_) {
      RCLCPP_INFO(this->get_logger(), "Pushing waypoints while DISARMED (this is allowed)");
    }
    
    return true;
  }

  void push_waypoints(const std::vector<sensor_msgs::msg::NavSatFix>& waypoints, 
                     double hold_time = 0.0, 
                     double acceptance_radius = 0.5,
                     const std::vector<double>& yaws = {}) {  // yaws in degrees
    
    if (!can_push_waypoints(waypoints)) {
      return;
    }

    publish_status("Pushing " + std::to_string(waypoints.size()) + " waypoints...");

    // Clear existing mission first for clean slate
    if (!clear_waypoints_sync()) {
      publish_status("Push waypoints aborted: failed to clear existing mission");
      return;
    }

    // Check if service is available
    if (!waypoint_push_client_->wait_for_service(std::chrono::seconds(2))) {
      publish_status("Push waypoints failed: MAVROS waypoint push service not available");
      return;
    }

    // Create service request
    auto request = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();
    request->start_index = 0;  // New mission, not appending

    // Check if yaws are provided and match waypoints count
    bool use_yaws = !yaws.empty() && yaws.size() == waypoints.size();
    if (!yaws.empty() && yaws.size() != waypoints.size()) {
      RCLCPP_WARN(this->get_logger(), 
                  "Yaw count (%zu) doesn't match waypoint count (%zu), ignoring yaws",
                  yaws.size(), waypoints.size());
      use_yaws = false;
    }

    // If on ground, prepend a MAV_CMD_NAV_TAKEOFF for clean takeoff profile
    bool added_takeoff = false;
    if (!airborne_ && !waypoints.empty()) {
      mavros_msgs::msg::Waypoint takeoff_wp;
      takeoff_wp.frame = mavros_msgs::msg::Waypoint::FRAME_GLOBAL_REL_ALT;
      takeoff_wp.command = 22;  // MAV_CMD_NAV_TAKEOFF
      takeoff_wp.is_current = true;  // Takeoff is the current waypoint
      takeoff_wp.autocontinue = true;
      takeoff_wp.param1 = 0.0;  // Minimum pitch (ignored by PX4 multicopters)
      takeoff_wp.param2 = 0.0;  // Empty
      takeoff_wp.param3 = 0.0;  // Empty  
      takeoff_wp.param4 = std::numeric_limits<double>::quiet_NaN();  // Yaw angle (use current)
      takeoff_wp.x_lat = 0.0;  // Latitude (ignored - takeoff at current position)
      takeoff_wp.y_long = 0.0;  // Longitude (ignored - takeoff at current position)
      takeoff_wp.z_alt = waypoints[0].altitude;  // Target altitude from first waypoint
      
      request->waypoints.push_back(takeoff_wp);
      added_takeoff = true;
      RCLCPP_INFO(this->get_logger(), "Added MAV_CMD_NAV_TAKEOFF to altitude %.1f m", waypoints[0].altitude);
    }

    // Convert NavSatFix waypoints to MAVROS waypoints
    for (size_t i = 0; i < waypoints.size(); i++) {
      mavros_msgs::msg::Waypoint wp;
      wp.frame = mavros_msgs::msg::Waypoint::FRAME_GLOBAL_REL_ALT;
      wp.command = 16;  // MAV_CMD_NAV_WAYPOINT
      wp.is_current = added_takeoff ? false : (i == 0);  // First nav waypoint is current only if no takeoff added
      wp.autocontinue = true;
      wp.param1 = hold_time;  // Hold time at waypoint
      wp.param2 = acceptance_radius;  // Acceptance radius
      wp.param3 = 0.0;  // Pass through waypoint
      wp.param4 = use_yaws ? yaws[i] : std::numeric_limits<double>::quiet_NaN();  // Yaw angle in degrees
      wp.x_lat = waypoints[i].latitude;
      wp.y_long = waypoints[i].longitude;
      wp.z_alt = waypoints[i].altitude;

      request->waypoints.push_back(wp);
    }

    // Call service asynchronously
    auto future = waypoint_push_client_->async_send_request(request);
    
    // Wait for response with timeout
    if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
      auto response = future.get();
      if (response->success) {
        publish_status("Waypoints pushed successfully (" + std::to_string(waypoints.size()) + " waypoints)");
        // Note: Caller should explicitly switch to AUTO.MISSION if needed
      } else {
        publish_status("Push waypoints failed: PX4 rejected waypoints");
      }
    } else {
      publish_status("Push waypoints timed out");
    }
  }

  // Waypoint service handler
  void handle_waypoint_service(
    const std::shared_ptr<behavior_governor::srv::PushWaypoints::Request> request,
    std::shared_ptr<behavior_governor::srv::PushWaypoints::Response> response) {
    
    RCLCPP_INFO(this->get_logger(), "Received waypoint push service request with %zu waypoints", 
                request->waypoints.size());

    // Validate request
    if (!can_push_waypoints(request->waypoints)) {
      response->success = false;
      response->message = "Waypoint validation failed - check status topic for details";
      response->waypoints_pushed = 0;
      return;
    }

    // Pass yaws directly (expecting degrees from service caller)
    // MAV_CMD_NAV_WAYPOINT param4 expects yaw in degrees
    
    // Call internal push_waypoints function with synchronous response
    bool success = push_waypoints_sync(request->waypoints, 
                                      request->hold_time, 
                                      request->acceptance_radius, 
                                      request->yaws);  // Pass degrees directly

    response->success = success;
    response->waypoints_pushed = success ? request->waypoints.size() : 0;
    response->message = success ? 
      "Waypoints pushed successfully" : 
      "Failed to push waypoints - check status topic for details";
  }

  // Synchronous version of push_waypoints for service calls
  bool push_waypoints_sync(const std::vector<sensor_msgs::msg::NavSatFix>& waypoints, 
                          double hold_time = 0.0, 
                          double acceptance_radius = 0.5,
                          const std::vector<double>& yaws = {}) {  // yaws in degrees
    
    if (!can_push_waypoints(waypoints)) {
      return false;
    }

    publish_status("Pushing " + std::to_string(waypoints.size()) + " waypoints...");

    // Clear existing mission first for clean slate
    if (!clear_waypoints_sync()) {
      publish_status("Push waypoints aborted: failed to clear existing mission");
      return false;
    }

    // Check if service is available
    if (!waypoint_push_client_->wait_for_service(std::chrono::seconds(2))) {
      publish_status("Push waypoints failed: MAVROS waypoint push service not available");
      return false;
    }

    // Create service request
    auto request = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();
    request->start_index = 0;  // New mission, not appending

    // Check if yaws are provided and match waypoints count
    bool use_yaws = !yaws.empty() && yaws.size() == waypoints.size();
    if (!yaws.empty() && yaws.size() != waypoints.size()) {
      RCLCPP_WARN(this->get_logger(), 
                  "Yaw count (%zu) doesn't match waypoint count (%zu), ignoring yaws",
                  yaws.size(), waypoints.size());
      use_yaws = false;
    }

    // If on ground, prepend a MAV_CMD_NAV_TAKEOFF for clean takeoff profile
    bool added_takeoff = false;
    if (!airborne_ && !waypoints.empty()) {
      mavros_msgs::msg::Waypoint takeoff_wp;
      takeoff_wp.frame = mavros_msgs::msg::Waypoint::FRAME_GLOBAL_REL_ALT;
      takeoff_wp.command = 22;  // MAV_CMD_NAV_TAKEOFF
      takeoff_wp.is_current = true;  // Takeoff is the current waypoint
      takeoff_wp.autocontinue = true;
      takeoff_wp.param1 = 0.0;  // Minimum pitch (ignored by PX4 multicopters)
      takeoff_wp.param2 = 0.0;  // Empty
      takeoff_wp.param3 = 0.0;  // Empty  
      takeoff_wp.param4 = std::numeric_limits<double>::quiet_NaN();  // Yaw angle (use current)
      takeoff_wp.x_lat = 0.0;  // Latitude (ignored - takeoff at current position)
      takeoff_wp.y_long = 0.0;  // Longitude (ignored - takeoff at current position)
      takeoff_wp.z_alt = waypoints[0].altitude;  // Target altitude from first waypoint
      
      request->waypoints.push_back(takeoff_wp);
      added_takeoff = true;
      RCLCPP_INFO(this->get_logger(), "Added MAV_CMD_NAV_TAKEOFF to altitude %.1f m", waypoints[0].altitude);
    }

    // Convert NavSatFix waypoints to MAVROS waypoints
    for (size_t i = 0; i < waypoints.size(); i++) {
      mavros_msgs::msg::Waypoint wp;
      wp.frame = mavros_msgs::msg::Waypoint::FRAME_GLOBAL_REL_ALT;
      wp.command = 16;  // MAV_CMD_NAV_WAYPOINT
      wp.is_current = added_takeoff ? false : (i == 0);  // First nav waypoint is current only if no takeoff added
      wp.autocontinue = true;
      wp.param1 = hold_time;  // Hold time at waypoint
      wp.param2 = acceptance_radius;  // Acceptance radius
      wp.param3 = 0.0;  // Pass through waypoint
      wp.param4 = use_yaws ? yaws[i] : std::numeric_limits<double>::quiet_NaN();  // Yaw angle in degrees
      wp.x_lat = waypoints[i].latitude;
      wp.y_long = waypoints[i].longitude;
      wp.z_alt = waypoints[i].altitude;

      request->waypoints.push_back(wp);
    }

    // Call service synchronously
    auto future = waypoint_push_client_->async_send_request(request);
    
    // Wait for response with timeout
    if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
      auto response = future.get();
      if (response->success) {
        publish_status("Waypoints pushed successfully (" + std::to_string(waypoints.size()) + " waypoints)");
        return true;
      } else {
        publish_status("Push waypoints failed: PX4 rejected waypoints");
        return false;
      }
    } else {
      publish_status("Push waypoints timed out");
      return false;
    }
  }

  // Safety check functions
  bool can_arm() {
    if (armed_) {
      publish_status("Arm rejected: already armed");
      return false;
    }
    if (airborne_) {
      publish_status("Arm rejected: vehicle appears to be airborne");
      return false;
    }
    return true;
  }

  bool can_disarm() {
    if (!armed_) {
      publish_status("Disarm rejected: already disarmed");
      return false;
    }
    if (airborne_) {
      publish_status("Disarm rejected: vehicle airborne - unsafe to disarm");
      return false;
    }
    return true;
  }

  // Arm command handler
  void handle_arm_command() {
    if (!can_arm()) {
      return;
    }

    publish_status("Attempting to arm vehicle...");

    // Check if service is available
    if (!arm_disarm_client_->wait_for_service(std::chrono::seconds(2))) {
      publish_status("Arm failed: MAVROS arming service not available");
      return;
    }

    // Create service request
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = true;

    // Call service asynchronously
    auto future = arm_disarm_client_->async_send_request(request);
    
    // Wait for response with timeout
    if (future.wait_for(std::chrono::seconds(3)) == std::future_status::ready) {
      auto response = future.get();
      if (response->success) {
        publish_status("Arm command sent successfully");
        // Note: actual armed state will be updated via state callback
      } else {
        publish_status("Arm command failed: " + std::to_string(response->result));
      }
    } else {
      publish_status("Arm command timed out");
    }
  }

  // Disarm command handler
  void handle_disarm_command() {
    if (!can_disarm()) {
      return;
    }

    publish_status("Attempting to disarm vehicle...");

    // Check if service is available
    if (!arm_disarm_client_->wait_for_service(std::chrono::seconds(2))) {
      publish_status("Disarm failed: MAVROS arming service not available");
      return;
    }

    // Create service request
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = false;

    // Call service asynchronously
    auto future = arm_disarm_client_->async_send_request(request);
    
    // Wait for response with timeout
    if (future.wait_for(std::chrono::seconds(3)) == std::future_status::ready) {
      auto response = future.get();
      if (response->success) {
        publish_status("Disarm command sent successfully");
        // Note: actual armed state will be updated via state callback
      } else {
        publish_status("Disarm command failed: " + std::to_string(response->result));
      }
    } else {
      publish_status("Disarm command timed out");
    }
  }

  // ROS2 interfaces
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr behavior_cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<mavros_msgs::msg::State>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr altitude_sub_;
  
  // MAVROS service clients
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_disarm_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr waypoint_push_client_;
  rclcpp::Client<mavros_msgs::srv::WaypointClear>::SharedPtr waypoint_clear_client_;
  
  // Service servers
  rclcpp::Service<behavior_governor::srv::PushWaypoints>::SharedPtr waypoint_service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  
  // Use MultiThreadedExecutor to prevent deadlock when calling services
  // from within callbacks. Single-threaded executors cannot process
  // service responses while blocked on future.wait_for() calls.
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<BehaviorGovernor>();
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}

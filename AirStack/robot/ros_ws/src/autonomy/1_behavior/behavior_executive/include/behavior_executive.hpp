#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <airstack_common/ros2_helper.hpp>
#include <airstack_msgs/srv/robot_command.hpp>
#include <airstack_msgs/srv/takeoff_landing_command.hpp>
#include <airstack_msgs/srv/trajectory_mode.hpp>
#include <behavior_tree/behavior_tree.hpp>
#include <behavior_tree_msgs/msg/behavior_tree_commands.hpp>
#include <mavros_msgs/msg/waypoint.hpp>
#include <mavros_msgs/msg/waypoint_list.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/waypoint_clear.hpp>
#include <mavros_msgs/srv/waypoint_push.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vector>
// #include <vision_msgs/msg/detection2_d_array.hpp>

#include "mavros_msgs/msg/state.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class BehaviorExecutive : public rclcpp::Node {
   private:
    int hold_active_id;

    // parameters
    std::string current_mode;
    double current_relative_altitude;
    int current_waypoint_index;
    double terrain_altitude;
    bool takeoff_altitude_set;
    double target_takeoff_altitude;
    double target_landing_altitude;
    double takeoff_altitude_threshold;
    double landing_altitude_threshold;
    double target_ascend_altitude;
    double ascend_altitude_threshold;
    double current_latitude;
    double current_longitude;
    double current_terrain_altitude;
    int8_t current_task_status;  // 0 for mapping mode, 1 for active searching mode
    struct {
        double latitude;
        double longitude;
    } target_waypoint;

    double min_confidence;  

    // Condition variables
    bt::Condition* auto_takeoff_commanded_condition;
    bt::Condition* armed_condition;
    bt::Condition* arm_commanded_condition;
    bt::Condition* disarm_commanded_condition;
    bt::Condition* e_stop_commanded_condition;
    bt::Condition* auto_land_commanded_condition;

    bt::Condition* geofence_mapping_commanded_condition;
    bt::Condition* search_commanded_condition;
    bt::Condition* survey_commanded_condition;
    std::vector<bt::Condition*> conditions;

    // Action variables
    bt::Action* arm_action;
    bt::Action* auto_takeoff_action;
    bt::Action* auto_land_action;
    bt::Action* hold_action;
    bt::Action* disarm_action;
    bt::Action* geofence_mapping_action;
    bt::Action* search_action;
    bt::Action* survey_action;
    std::vector<bt::Action*> actions;

    // subscribers
    rclcpp::Subscription<behavior_tree_msgs::msg::BehaviorTreeCommands>::SharedPtr
        behavior_tree_commands_sub;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr relative_altitude_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr set_target_altitude_sub;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr target_waypoint_sub;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr current_gps_sub;
    rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr geofence_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr drone_pose_sub;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr detection_box_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gps_deconflict_sub;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr target_gps_list_sub;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr target_gps_sub;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr precise_target_gps_sub;

    // rclcpp::Subscription<vision_msgs::Detection2DArray>::SharedPtr detection_box_sub_;
    // publishers
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr task_status_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gimbal_command_pub;

    // services
    rclcpp::CallbackGroup::SharedPtr service_callback_group;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_disarm_command_client;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_command_client;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_command_client;
    rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr waypoint_push_client;
    rclcpp::Client<mavros_msgs::srv::WaypointClear>::SharedPtr waypoint_clear_client;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr global_planner_toggle_client;

    // timers
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr completion_timer;
    rclcpp::TimerBase::SharedPtr timeout_timer;
    rclcpp::TimerBase::SharedPtr search_timer;
    // callbacks
    void timer_callback();
    void bt_commands_callback(behavior_tree_msgs::msg::BehaviorTreeCommands msg);
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg);
    void relative_altitude_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void set_target_altitude_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void target_waypoint_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void current_gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void geofence_callback(const mavros_msgs::msg::WaypointList::SharedPtr msg);
    void imu_data_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void UAV_Deconflict_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void target_gps_list_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void target_gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void precise_target_gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    // void detectionBoxCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);
    // void detectionBoxCallback(const vision_msgs::Detection2DArray::SharedPtr msg);

    // helper functions
    std::vector<sensor_msgs::msg::NavSatFix> generate_hexagon_waypoints(double center_lat,
                                                                        double center_lon,
                                                                        double radius, double theta,
                                                                        double altitude);

    // Add the point_to_line_segment_distance function
    double point_to_line_segment_distance(
        const std::pair<double, double>& point, const std::pair<double, double>& line_start,
        const std::pair<double, double>& line_end);

    // Add the is_point_safe_distance function if it doesn't exist in the .cpp file
    bool is_point_safe_distance(const std::pair<double, double>& point,
                               const std::vector<std::pair<double, double>>& fence,
                               double safe_dist_lat, double safe_dist_lon);
                               
    // Helper function to calculate the cross product of three points for convex hull
    double cross_product(const std::pair<double, double>& a,
                      const std::pair<double, double>& b,
                      const std::pair<double, double>& c);

    // Function to compute the convex hull using the Graham scan algorithm
    std::vector<std::pair<double, double>> convex_hull(std::vector<std::pair<double, double>>& points);

    void set_mode(bt::Action* action,
                  const rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr& client,
                  const std::string& mode);

    void push_waypoints(bt::Action* action,
                        const std::vector<sensor_msgs::msg::NavSatFix>& waypoints,
                        double target_altitude, double hold_time = 0.0,
                        const std::vector<double>& yaws = {}, double acceptance_radius=0.5);

    void process_next_hexagon_waypoint();

    static bool is_point_in_fence(const std::pair<double, double>& point,
                                  const std::vector<std::pair<double, double>>& fence_corners);

    // Check if a waypoint is a valid geofence point (command 5001 indicates a geofence vertex)
    static bool is_geofence_waypoint(const mavros_msgs::msg::Waypoint& waypoint);

    std::vector<std::pair<double, double>> geofence_points;
    bool geofence_active = false;

    // Orientation variables
    double current_roll;
    double current_pitch;
    double current_yaw;
    double current_heading;

    // Subscription for detection boxes
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr detection_box_sub_;

    // Hexagon navigation variables
    bool is_hexagon_in_progress;
    std::vector<sensor_msgs::msg::NavSatFix> hexagon_waypoints;
    rclcpp::TimerBase::SharedPtr hexagon_timer;

    // Casualty detection
    std::array<double, 2> casualty_gps_position;
    std::array<double, 2> precise_casualty_gps_position;
    bool precise_casualty_detected = false;

    void detectionBoxCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);

    // Search behavior
    bool is_search_in_progress;
    std::vector<sensor_msgs::msg::NavSatFix> search_waypoints;
    int search_current_waypoint_index;
    sensor_msgs::msg::NavSatFix target_gps;
    bool casualty_detected;

    void process_next_search_waypoint();
    std::vector<sensor_msgs::msg::NavSatFix> generate_rectangle_waypoints(double center_lat,
                                                                          double center_lon,
                                                                          double width_meters,
                                                                          double height_meters,
                                                                          double altitude);
    std::vector<sensor_msgs::msg::NavSatFix> generate_circle_waypoints(double center_lat,
                                                                       double center_lon,
                                                                       double radius_meters,
                                                                       int num_points,
                                                                       double altitude);

   public:
    BehaviorExecutive();
};

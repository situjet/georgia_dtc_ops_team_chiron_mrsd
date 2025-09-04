// behavior_governor.cpp
// C++ node for core vehicle behaviors, exposes topics/services for Python extension
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

using std::placeholders::_1;


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

    // Example: publisher for waypoint
    waypoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "mavros/setpoint_position/local", 10);

    // State subscriber (simulate getting vehicle state)
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "mavros/state", 10,
      std::bind(&BehaviorGovernor::on_state, this, _1));

    // Initialize state
    armed_ = false;
    mode_ = "";
    airborne_ = false;
  }

private:
  // State variables
  bool armed_;
  std::string mode_;
  bool airborne_;

  void on_state(const mavros_msgs::msg::State::SharedPtr msg) {
    armed_ = msg->armed;
    mode_ = msg->mode;
    // Optionally, update airborne_ based on altitude or other sensors
  }

  void on_behavior_command(const std_msgs::msg::String::SharedPtr msg) {
    // Parse command and validate conditions before state switch
    if (msg->data == "takeoff") {
      if (!armed_) {
        publish_status("Takeoff rejected: vehicle not armed");
        return;
      }
      if (airborne_) {
        publish_status("Takeoff rejected: already airborne");
        return;
      }
      // All conditions met, proceed to takeoff
      publish_status("Takeoff command accepted");
      // ... (call takeoff service or publish setpoint)
      airborne_ = true; // Simulate state change
    } else if (msg->data == "arm") {
      if (armed_) {
        publish_status("Arm rejected: already armed");
        return;
      }
      // All conditions met, proceed to arm
      publish_status("Arming vehicle");
      // ... (call arming service)
      armed_ = true; // Simulate state change
    } else if (msg->data == "disarm") {
      if (!armed_) {
        publish_status("Disarm rejected: already disarmed");
        return;
      }
      if (airborne_) {
        publish_status("Disarm rejected: vehicle airborne");
        return;
      }
      publish_status("Disarming vehicle");
      // ... (call disarm service)
      armed_ = false;
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
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr behavior_cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BehaviorGovernor>());
  rclcpp::shutdown();
  return 0;
}

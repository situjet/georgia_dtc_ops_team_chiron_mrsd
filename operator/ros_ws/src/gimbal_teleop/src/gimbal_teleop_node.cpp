#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3.h>

class GimbalTeleop
{
public:
    GimbalTeleop()
    {
        ros::NodeHandle nh;
        joy_sub_ = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &GimbalTeleop::joyCallback, this);
        gimbal_pub_ = nh.advertise<geometry_msgs::Vector3>("/gimbal_angles", 10);
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
    {
        geometry_msgs::Vector3 cmd;
        // Example mapping: right stick X (axes[3]) = yaw, right stick Y (axes[4]) = pitch
        // Triggers (axes[2], axes[5]) for zoom (if needed)
        cmd.x = joy->axes[4]; // pitch
        cmd.y = joy->axes[3]; // yaw
        cmd.z = (joy->axes[5] - joy->axes[2]) * 0.5; // zoom (example)
        gimbal_pub_.publish(cmd);
    }

private:
    ros::Subscriber joy_sub_;
    ros::Publisher gimbal_pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gimbal_teleop_node");
    GimbalTeleop teleop;
    ros::spin();
    return 0;
}

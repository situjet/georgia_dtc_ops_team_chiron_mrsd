#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/waypoint_clear.hpp>
#include <mavros_msgs/srv/waypoint_push.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace bt {
class Action;  // Forward declaration
}

/**
 * @brief MAVROS 适配器接口：封装所有 MAVROS 的 I/O 操作
 * 
 * 此接口将所有外部副作用操作（ROS 服务调用、消息发布等）从业务逻辑中分离出来，
 * 使得状态机逻辑可以独立测试，并通过依赖注入实现解耦。
 */
class IMavrosAdapter {
public:
    virtual ~IMavrosAdapter() = default;

    /**
     * @brief 设置飞行模式的回调类型
     * @param success 是否成功设置模式
     */
    using SetModeCallback = std::function<void(bool success)>;

    /**
     * @brief Arm/Disarm 操作的回调类型
     * @param success 是否成功执行操作
     */
    using ArmCallback = std::function<void(bool success)>;

    /**
     * @brief 航点操作的回调类型
     * @param success 是否成功执行操作
     */
    using WaypointCallback = std::function<void(bool success)>;

    /**
     * @brief 设置飞行模式
     * @param mode 目标模式字符串（如 "AUTO.TAKEOFF", "AUTO.LAND" 等）
     * @param callback 异步回调函数
     * @param logger_name 日志名称，用于日志输出
     */
    virtual void set_mode(const std::string& mode, 
                         SetModeCallback callback,
                         const std::string& logger_name) = 0;

    /**
     * @brief 武装飞行器
     * @param callback 异步回调函数
     * @param logger_name 日志名称
     */
    virtual void arm_vehicle(ArmCallback callback,
                            const std::string& logger_name) = 0;

    /**
     * @brief 解除飞行器武装
     * @param callback 异步回调函数
     * @param logger_name 日志名称
     */
    virtual void disarm_vehicle(ArmCallback callback,
                               const std::string& logger_name) = 0;

    /**
     * @brief 清除航点
     * @param callback 异步回调函数
     * @param logger_name 日志名称
     */
    virtual void clear_waypoints(WaypointCallback callback,
                                const std::string& logger_name) = 0;

    /**
     * @brief 推送航点
     * @param waypoints 航点列表
     * @param target_altitude 目标高度
     * @param hold_time 悬停时间
     * @param yaws 偏航角列表（可选）
     * @param acceptance_radius 接受半径
     * @param callback 异步回调函数
     * @param logger_name 日志名称
     * @param current_mode 当前飞行模式，用于判断是否需要切换到 MISSION 模式
     */
    virtual void push_waypoints(const std::vector<sensor_msgs::msg::NavSatFix>& waypoints,
                               double target_altitude,
                               double hold_time,
                               const std::vector<double>& yaws,
                               double acceptance_radius,
                               WaypointCallback callback,
                               const std::string& logger_name,
                               const std::string& current_mode) = 0;

    /**
     * @brief 发布云台命令
     * @param command 命令字符串（如 "LOCK_ON"）
     * @param count 发送次数
     */
    virtual void publish_gimbal_command(const std::string& command, int count = 1) = 0;

    /**
     * @brief 发布 Hold Active 消息（用于冲突解决）
     * @param id 消息ID
     */
    virtual void publish_hold_active(int id) = 0;
};


#pragma once

#include "action_handlers/action_handler_interface.hpp"
#include <memory>
#include <array>
#include <vector>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// Forward declarations
class IMavrosAdapter;

#include <rclcpp/rclcpp.hpp>

/**
 * @brief Survey 动作处理器
 * 
 * 封装巡视任务的完整逻辑：
 * 1. 接收目标位置（precise_casualty_gps_position 或默认位置）
 * 2. 生成六边形巡视航点
 * 3. 推送航点和偏航角到飞控
 * 4. 发送云台锁定命令
 * 
 * 注意：这是一个存档的 action，可能不会在生产环境中使用
 */
class SurveyHandler : public IActionHandler {
public:
    /**
     * @brief 构造函数（依赖注入）
     * @param mavros_adapter MAVROS 通信适配器
     * @param logger ROS logger
     */
    SurveyHandler(
        std::shared_ptr<IMavrosAdapter> mavros_adapter,
        rclcpp::Logger logger);

    /**
     * @brief 设置目标位置
     * @param casualty_position 目标位置 [lat, lon]
     * @param use_precise 是否使用精确位置
     */
    void set_target_position(const std::array<double, 2>& casualty_position, bool use_precise);

    /**
     * @brief 重置精确检测标志
     */
    void reset_precise_flag();

    // 实现 IActionHandler 接口
    void on_activated(bt::Action* action, const Context& ctx) override;
    bool check_safety(const Context& ctx) override;

private:
    std::shared_ptr<IMavrosAdapter> mavros_adapter_;
    rclcpp::Logger logger_;

    // 目标位置
    std::array<double, 2> target_position_;
    bool use_precise_position_;

    // 任务参数
    static constexpr double SURVEY_RADIUS = 4.5;            // 米：六边形半径
    static constexpr double SURVEY_ALTITUDE = 8.0;          // 米：巡视高度
    static constexpr double HOLD_TIME = 10.0;               // 秒：在航点停留时间
    static constexpr double ACCEPTANCE_RADIUS = 0.3;        // 米：航点接受半径
    static constexpr double INITIAL_HEADING = 270.0;        // 度：初始航向角
    static constexpr double HEADING_DECREMENT = 60.0;       // 度：每个顶点的航向角递减

    // 默认位置（如果没有接收到目标位置）
    static constexpr double DEFAULT_LAT = 40.4779091;
    static constexpr double DEFAULT_LON = -79.8927102;

    /**
     * @brief 生成六边形巡视航点
     * @param center_lat 中心纬度
     * @param center_lon 中心经度
     * @param radius 半径（米）
     * @param altitude 高度（米）
     * @param yaws 输出：每个航点的偏航角
     * @return 航点列表
     */
    std::vector<sensor_msgs::msg::NavSatFix> generate_hexagon_waypoints(
        double center_lat, double center_lon, double radius,
        double altitude, std::vector<double>& yaws);
};


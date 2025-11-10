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
 * @brief Search 动作处理器
 * 
 * 封装搜索任务的完整逻辑：
 * 1. 接收目标位置（casualty_gps_position 或 precise_casualty_gps_position）
 * 2. 生成圆形搜索航点
 * 3. 推送航点到飞控
 * 4. 切换到 AUTO.MISSION 模式
 * 
 * 注意：这是一个存档的 action，可能不会在生产环境中使用
 */
class SearchHandler : public IActionHandler {
public:
    /**
     * @brief 构造函数（依赖注入）
     * @param mavros_adapter MAVROS 通信适配器
     * @param logger ROS logger
     */
    SearchHandler(
        std::shared_ptr<IMavrosAdapter> mavros_adapter,
        rclcpp::Logger logger);

    /**
     * @brief 设置目标位置
     * @param casualty_position 目标位置 [lat, lon]
     * @param use_precise 是否使用精确位置
     */
    void set_target_position(const std::array<double, 2>& casualty_position, bool use_precise);

    /**
     * @brief 获取当前设置的目标位置
     * @return 目标位置 [lat, lon]
     */
    const std::array<double, 2>& get_target_position() const;

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
    static constexpr double SEARCH_RADIUS = 4.5;           // 米：搜索半径
    static constexpr int PERIMETER_POINTS = 24;            // 圆周上的航点数量 (25-1)
    static constexpr double SEARCH_ALTITUDE = 10.0;        // 米：搜索高度
    static constexpr double HOLD_TIME = 2.0;               // 秒：在航点停留时间
    static constexpr double ACCEPTANCE_RADIUS = 0.5;       // 米：航点接受半径

    /**
     * @brief 生成圆形搜索航点
     * @param center_lat 中心纬度
     * @param center_lon 中心经度
     * @param radius 半径（米）
     * @param num_points 航点数量
     * @param altitude 高度（米）
     * @return 航点列表
     */
    std::vector<sensor_msgs::msg::NavSatFix> generate_circle_waypoints(
        double center_lat, double center_lon, double radius,
        int num_points, double altitude);
};


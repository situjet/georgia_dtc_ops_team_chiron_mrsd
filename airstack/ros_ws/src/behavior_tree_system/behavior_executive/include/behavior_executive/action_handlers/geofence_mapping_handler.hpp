#pragma once

#include "action_handlers/action_handler_interface.hpp"
#include <memory>
#include <utility>  // For std::pair
#include <vector>

// Forward declarations
class IMavrosAdapter;
class WaypointGenerator;
class GeometryUtils;

#include <rclcpp/rclcpp.hpp>

/**
 * @brief Geofence Mapping 动作处理器
 * 
 * 封装地理围栏测绘任务的完整逻辑：
 * 1. 使用 GeometryUtils 计算凸包
 * 2. 使用 WaypointGenerator 生成割草机模式航点
 * 3. 通过 MavrosAdapter 推送航点到飞控
 */
class GeofenceMappingHandler : public IActionHandler {
public:
    /**
     * @brief 构造函数（依赖注入）
     * @param mavros_adapter MAVROS 通信适配器
     * @param waypoint_generator 航点生成器
     * @param geometry_utils 几何工具类
     * @param logger ROS logger
     */
    GeofenceMappingHandler(
        std::shared_ptr<IMavrosAdapter> mavros_adapter,
        std::shared_ptr<WaypointGenerator> waypoint_generator,
        std::shared_ptr<GeometryUtils> geometry_utils,
        rclcpp::Logger logger);

    /**
     * @brief 设置地理围栏点
     * @param points 围栏顶点列表 (lat, lon)
     */
    void set_geofence_points(const std::vector<std::pair<double, double>>& points);

    /**
     * @brief 获取当前的地理围栏点
     * @return 围栏顶点列表
     */
    const std::vector<std::pair<double, double>>& get_geofence_points() const;

    // 实现 IActionHandler 接口
    void on_activated(bt::Action* action, const Context& ctx) override;
    bool check_safety(const Context& ctx) override;

private:
    std::shared_ptr<IMavrosAdapter> mavros_adapter_;
    std::shared_ptr<WaypointGenerator> waypoint_generator_;
    std::shared_ptr<GeometryUtils> geometry_utils_;
    rclcpp::Logger logger_;

    // 地理围栏点（从 /robot_1/mavros/geofence/fences 接收）
    std::vector<std::pair<double, double>> geofence_points_;

    // 任务参数（可以通过配置文件或参数设置）
    static constexpr double FOOTPRINT_WIDTH = 5.0;        // 米
    static constexpr double OVERLAP_PERCENTAGE = 10.0;    // 百分比
    static constexpr double SAFE_DISTANCE = 1.0;          // 米
    static constexpr double HOLD_TIME = 0.0;              // 秒
    static constexpr double ACCEPTANCE_RADIUS = 0.3;      // 米
};


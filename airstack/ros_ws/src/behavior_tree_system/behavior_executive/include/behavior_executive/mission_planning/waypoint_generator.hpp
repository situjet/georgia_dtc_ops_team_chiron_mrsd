#pragma once

#include <memory>
#include <utility>  // For std::pair
#include <vector>

#include <sensor_msgs/msg/nav_sat_fix.hpp>

// Forward declaration
class GeometryUtils;

/**
 * @brief 航点生成器：用于生成各种模式的任务航点
 * 
 * 此类封装了航点生成算法，包括割草机模式、网格搜索等。
 * 使用依赖注入 GeometryUtils 进行几何计算。
 */
class WaypointGenerator {
public:
    /**
     * @brief 构造函数
     * @param geometry_utils 几何工具类实例（依赖注入）
     */
    explicit WaypointGenerator(std::shared_ptr<GeometryUtils> geometry_utils);

    /**
     * @brief 生成割草机模式的航点，覆盖给定的地理围栏区域
     * 
     * @param geofence_corners 围栏顶点（已计算凸包）
     * @param current_lat 当前无人机纬度
     * @param current_lon 当前无人机经度
     * @param footprint_width 相机/传感器覆盖宽度（米）
     * @param overlap_percentage 相邻航线重叠百分比 (0-100)
     * @param safe_distance 距离围栏边界的安全距离（米）
     * @param target_altitude 目标飞行高度（米）
     * @return 航点列表（NavSatFix格式）
     */
    std::vector<sensor_msgs::msg::NavSatFix> generate_lawnmower_pattern(
        const std::vector<std::pair<double, double>>& geofence_corners,
        double current_lat,
        double current_lon,
        double footprint_width,
        double overlap_percentage,
        double safe_distance,
        double target_altitude);

private:
    std::shared_ptr<GeometryUtils> geometry_utils_;
};





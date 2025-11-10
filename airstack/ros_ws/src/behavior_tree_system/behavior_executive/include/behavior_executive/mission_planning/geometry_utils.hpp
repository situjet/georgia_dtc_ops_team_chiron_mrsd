#ifndef GEOMETRY_UTILS_HPP
#define GEOMETRY_UTILS_HPP

#include <vector>
#include <utility>  // for std::pair

/**
 * @brief 几何计算工具类
 * 
 * 提供纯函数的几何计算功能，包括凸包计算、点在多边形内检测等。
 * 所有方法都是静态的，不依赖于对象状态。
 */
class GeometryUtils {
public:
    /**
     * @brief 计算叉积
     * @param a 第一个点
     * @param b 第二个点
     * @param c 第三个点
     * @return 叉积值：正值表示逆时针，负值表示顺时针，零表示共线
     */
    static double cross_product(const std::pair<double, double>& a,
                               const std::pair<double, double>& b,
                               const std::pair<double, double>& c);

    /**
     * @brief 计算凸包（Graham扫描算法）
     * @param points 输入点集
     * @return 凸包顶点列表（逆时针顺序）
     */
    static std::vector<std::pair<double, double>> convex_hull(
        std::vector<std::pair<double, double>>& points);

    /**
     * @brief 检查点是否在多边形内（射线投射算法）
     * @param point 测试点
     * @param fence 多边形顶点列表
     * @return true 如果点在多边形内
     */
    static bool is_point_in_fence(const std::pair<double, double>& point,
                                  const std::vector<std::pair<double, double>>& fence);

    /**
     * @brief 计算点到线段的距离
     * @param point 测试点
     * @param line_start 线段起点
     * @param line_end 线段终点
     * @return 距离值
     */
    static double point_to_line_segment_distance(const std::pair<double, double>& point,
                                                 const std::pair<double, double>& line_start,
                                                 const std::pair<double, double>& line_end);

    /**
     * @brief 检查点是否距离地理围栏边界安全距离
     * @param point 测试点
     * @param fence 地理围栏顶点列表
     * @param safe_dist_lat 纬度方向安全距离阈值
     * @param safe_dist_lon 经度方向安全距离阈值
     * @return true 如果点在安全距离内
     */
    static bool is_point_safe_distance(const std::pair<double, double>& point,
                                       const std::vector<std::pair<double, double>>& fence,
                                       double safe_dist_lat,
                                       double safe_dist_lon);
};

#endif // GEOMETRY_UTILS_HPP


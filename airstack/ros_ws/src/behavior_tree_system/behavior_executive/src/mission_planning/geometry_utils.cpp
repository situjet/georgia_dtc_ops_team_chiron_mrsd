#include "mission_planning/geometry_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

double GeometryUtils::cross_product(const std::pair<double, double>& a,
                                    const std::pair<double, double>& b,
                                    const std::pair<double, double>& c) {
    return (b.first - a.first) * (c.second - a.second) -
           (b.second - a.second) * (c.first - a.first);
}

std::vector<std::pair<double, double>> GeometryUtils::convex_hull(
    std::vector<std::pair<double, double>>& points) {
    int n = points.size();
    if (n <= 2) return points;

    // 1. 找到y坐标最小的点（平局时选择x最小的）
    auto lowest_y_it = std::min_element(
        points.begin(), points.end(),
        [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
            if (a.second != b.second) {
                return a.second < b.second;
            }
            return a.first < b.first;
        });
    std::iter_swap(points.begin(), lowest_y_it);
    std::pair<double, double> pivot = points[0];

    // 2. 根据极角排序剩余点
    std::sort(
        points.begin() + 1, points.end(),
        [&](const std::pair<double, double>& a, const std::pair<double, double>& b) {
            double orientation = cross_product(pivot, a, b);
            if (orientation == 0) {
                // 如果共线，按距离排序
                return std::pow(a.first - pivot.first, 2) + std::pow(a.second - pivot.second, 2) <
                       std::pow(b.first - pivot.first, 2) + std::pow(b.second - pivot.second, 2);
            }
            return orientation > 0;  // 逆时针顺序
        });

    // 3. 构建凸包
    std::vector<std::pair<double, double>> hull;
    hull.push_back(points[0]);
    hull.push_back(points[1]);

    for (int i = 2; i < n; ++i) {
        while (hull.size() >= 2 &&
               cross_product(hull[hull.size() - 2], hull.back(), points[i]) <= 0) {
            hull.pop_back();
        }
        hull.push_back(points[i]);
    }

    return hull;
}

bool GeometryUtils::is_point_in_fence(const std::pair<double, double>& point,
                                      const std::vector<std::pair<double, double>>& fence) {
    // 射线投射算法实现
    int num_vertices = fence.size();
    bool inside = false;

    // 测试点坐标
    double testx = point.first;
    double testy = point.second;

    // 遍历多边形的所有边
    for (int i = 0, j = num_vertices - 1; i < num_vertices; j = i++) {
        // 顶点坐标
        double vertx_i = fence[i].first;
        double verty_i = fence[i].second;
        double vertx_j = fence[j].first;
        double verty_j = fence[j].second;

        // 检查从点发出的水平射线是否穿过这条边
        if (((verty_i > testy) != (verty_j > testy)) &&
            (testx < (vertx_j - vertx_i) * (testy - verty_i) / (verty_j - verty_i) + vertx_i)) {
            // 切换内外状态
            inside = !inside;
        }
    }

    return inside;
}

double GeometryUtils::point_to_line_segment_distance(
    const std::pair<double, double>& point,
    const std::pair<double, double>& line_start,
    const std::pair<double, double>& line_end) {
    // 转换为 x,y 表示法以便清晰
    double x = point.first;
    double y = point.second;
    double x1 = line_start.first;
    double y1 = line_start.second;
    double x2 = line_end.first;
    double y2 = line_end.second;

    // 计算线段的平方长度
    double line_length_squared = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

    // 如果线段实际上是一个点，返回到该点的距离
    if (line_length_squared == 0.0) {
        return std::sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
    }

    // 计算点在线段上的投影
    double t = std::max(
        0.0, std::min(1.0, ((x - x1) * (x2 - x1) + (y - y1) * (y2 - y1)) / line_length_squared));

    // 计算线段上最近的点
    double closest_x = x1 + t * (x2 - x1);
    double closest_y = y1 + t * (y2 - y1);

    // 返回到最近点的距离
    return std::sqrt((x - closest_x) * (x - closest_x) + (y - closest_y) * (y - closest_y));
}

bool GeometryUtils::is_point_safe_distance(const std::pair<double, double>& point,
                                           const std::vector<std::pair<double, double>>& fence,
                                           double safe_dist_lat,
                                           double safe_dist_lon) {
    // 如果围栏少于2个点，无法检查距离
    if (fence.size() < 2) {
        return true;  // 假定安全，因为没有真正的围栏
    }

    double min_distance = std::numeric_limits<double>::max();

    // 检查到每条围栏边的距离
    for (size_t i = 0; i < fence.size(); i++) {
        size_t j = (i + 1) % fence.size();  // 获取下一个点，环绕到起点

        double dist = point_to_line_segment_distance(point, fence[i], fence[j]);
        min_distance = std::min(min_distance, dist);
    }

    // 以度为单位比较距离
    // 注意：这里使用 safe_dist_lat 作为主要阈值
    return min_distance >= safe_dist_lat;
}





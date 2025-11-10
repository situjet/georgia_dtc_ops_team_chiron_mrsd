#include "mission_planning/waypoint_generator.hpp"
#include "mission_planning/geometry_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

// 常量定义
constexpr double METERS_PER_LAT_DEG = 111111.0;  // 1度纬度约等于111111米

WaypointGenerator::WaypointGenerator(std::shared_ptr<GeometryUtils> geometry_utils)
    : geometry_utils_(geometry_utils) {}

std::vector<sensor_msgs::msg::NavSatFix> WaypointGenerator::generate_lawnmower_pattern(
    const std::vector<std::pair<double, double>>& geofence_corners,
    double current_lat,
    double current_lon,
    double footprint_width,
    double overlap_percentage,
    double safe_distance,
    double target_altitude) {
    
    std::vector<sensor_msgs::msg::NavSatFix> waypoints;

    if (geofence_corners.size() < 3) {
        return waypoints;  // 返回空列表，需要至少3个点
    }

    // 计算围栏中心用于转换因子
    double center_lat = 0.0;
    double center_lon = 0.0;
    for (const auto& corner : geofence_corners) {
        center_lat += corner.first;
        center_lon += corner.second;
    }
    center_lat /= geofence_corners.size();
    center_lon /= geofence_corners.size();

    // 米到度的转换因子
    double meters_per_deg_lat = METERS_PER_LAT_DEG;
    double meters_per_deg_lon = METERS_PER_LAT_DEG * std::cos(center_lat * M_PI / 180.0);

    // 计算安全距离（度）
    double safe_dist_lat = safe_distance / meters_per_deg_lat;
    double safe_dist_lon = safe_distance / meters_per_deg_lon;

    // 计算边界框
    double min_lat = std::numeric_limits<double>::max();
    double max_lat = std::numeric_limits<double>::lowest();
    double min_lon = std::numeric_limits<double>::max();
    double max_lon = std::numeric_limits<double>::lowest();

    for (const auto& corner : geofence_corners) {
        min_lat = std::min(min_lat, corner.first);
        max_lat = std::max(max_lat, corner.first);
        min_lon = std::min(min_lon, corner.second);
        max_lon = std::max(max_lon, corner.second);
    }

    // 计算基于覆盖宽度和重叠的步长
    double footprint_width_deg_lat = footprint_width / meters_per_deg_lat;
    double footprint_width_deg_lon = footprint_width / meters_per_deg_lon;

    double overlap_fraction = overlap_percentage / 100.0;
    double step_size_lat = footprint_width_deg_lat * (1.0 - overlap_fraction);
    double step_size_lon = footprint_width_deg_lon * (1.0 - overlap_fraction);

    // 确保步长为正
    if (step_size_lat <= 0 || step_size_lon <= 0) {
        return waypoints;  // 无效参数
    }

    // 计算基于完整边界框的行列数
    int num_passes_lon = std::max(1, static_cast<int>(std::ceil((max_lon - min_lon) / step_size_lon)));
    int num_passes_lat = std::max(1, static_cast<int>(std::ceil((max_lat - min_lat) / step_size_lat)));

    // 将当前位置对齐到最近的纬度步长
    double snapped_lat = std::round((current_lat - min_lat) / step_size_lat) * step_size_lat + min_lat;
    int start_lat_idx = std::round((snapped_lat - min_lat) / step_size_lat);
    start_lat_idx = std::max(0, std::min(start_lat_idx, num_passes_lat - 1));

    // 将当前位置对齐到最近的经度步长
    std::vector<double> lon_array;
    for (int j = 0; j < num_passes_lon; j++) {
        lon_array.push_back(min_lon + j * step_size_lon);
    }
    
    int start_lon_idx = 0;
    if (!lon_array.empty()) {
        double closest_lon = lon_array[0];
        double min_diff = std::abs(current_lon - closest_lon);
        for (size_t j = 1; j < lon_array.size(); ++j) {
            double diff = std::abs(current_lon - lon_array[j]);
            if (diff < min_diff) {
                min_diff = diff;
                closest_lon = lon_array[j];
                start_lon_idx = j;
            }
        }
    }
    start_lon_idx = std::max(0, std::min(start_lon_idx, num_passes_lon - 1));

    // 生成割草机模式的航点
    bool going_right = true;
    std::vector<bool> row_processed(num_passes_lat, false);

    // 处理起始纬度行
    if (start_lat_idx >= 0 && start_lat_idx < num_passes_lat) {
        double lat = min_lat + start_lat_idx * step_size_lat;
        std::vector<std::pair<double, double>> current_row_waypoints;
        
        if (going_right) {
            // 从对齐的经度索引向右开始
            for (int j = start_lon_idx; j < num_passes_lon; j++) {
                std::pair<double, double> wp = {lat, min_lon + j * step_size_lon};
                if (geometry_utils_->is_point_in_fence(wp, geofence_corners) &&
                    geometry_utils_->is_point_safe_distance(wp, geofence_corners, safe_dist_lat, safe_dist_lon)) {
                    current_row_waypoints.push_back(wp);
                }
            }
            // 然后向左扫描该行剩余的点
            for (int j = start_lon_idx - 1; j >= 0; --j) {
                std::pair<double, double> wp = {lat, min_lon + j * step_size_lon};
                if (geometry_utils_->is_point_in_fence(wp, geofence_corners) &&
                    geometry_utils_->is_point_safe_distance(wp, geofence_corners, safe_dist_lat, safe_dist_lon)) {
                    current_row_waypoints.push_back(wp);
                }
            }
        } else {
            // 从对齐的经度索引向左开始
            for (int j = start_lon_idx; j >= 0; j--) {
                std::pair<double, double> wp = {lat, min_lon + j * step_size_lon};
                if (geometry_utils_->is_point_in_fence(wp, geofence_corners) &&
                    geometry_utils_->is_point_safe_distance(wp, geofence_corners, safe_dist_lat, safe_dist_lon)) {
                    current_row_waypoints.push_back(wp);
                }
            }
            // 然后向右扫描剩余点
            for (int j = start_lon_idx + 1; j < num_passes_lon; ++j) {
                std::pair<double, double> wp = {lat, min_lon + j * step_size_lon};
                if (geometry_utils_->is_point_in_fence(wp, geofence_corners) &&
                    geometry_utils_->is_point_safe_distance(wp, geofence_corners, safe_dist_lat, safe_dist_lon)) {
                    current_row_waypoints.push_back(wp);
                }
            }
        }
        
        waypoints.reserve(waypoints.size() + current_row_waypoints.size());
        for (const auto& wp_pair : current_row_waypoints) {
            sensor_msgs::msg::NavSatFix wp;
            wp.latitude = wp_pair.first;
            wp.longitude = wp_pair.second;
            wp.altitude = target_altitude;
            waypoints.push_back(wp);
        }
        
        row_processed[start_lat_idx] = true;
        going_right = !going_right;
    }

    // 处理起始行上方的行
    for (int i = start_lat_idx + 1; i < num_passes_lat; i++) {
        if (row_processed[i]) continue;
        
        double lat = min_lat + i * step_size_lat;
        std::vector<std::pair<double, double>> current_row_waypoints;
        
        if (going_right) {
            for (int j = 0; j < num_passes_lon; j++) {
                std::pair<double, double> wp = {lat, min_lon + j * step_size_lon};
                if (geometry_utils_->is_point_in_fence(wp, geofence_corners) &&
                    geometry_utils_->is_point_safe_distance(wp, geofence_corners, safe_dist_lat, safe_dist_lon)) {
                    current_row_waypoints.push_back(wp);
                }
            }
        } else {
            for (int j = num_passes_lon - 1; j >= 0; j--) {
                std::pair<double, double> wp = {lat, min_lon + j * step_size_lon};
                if (geometry_utils_->is_point_in_fence(wp, geofence_corners) &&
                    geometry_utils_->is_point_safe_distance(wp, geofence_corners, safe_dist_lat, safe_dist_lon)) {
                    current_row_waypoints.push_back(wp);
                }
            }
        }
        
        if (!current_row_waypoints.empty()) {
            for (const auto& wp_pair : current_row_waypoints) {
                sensor_msgs::msg::NavSatFix wp;
                wp.latitude = wp_pair.first;
                wp.longitude = wp_pair.second;
                wp.altitude = target_altitude;
                waypoints.push_back(wp);
            }
            row_processed[i] = true;
            going_right = !going_right;
        }
    }

    // 处理起始行下方的行
    for (int i = start_lat_idx - 1; i >= 0; i--) {
        if (row_processed[i]) continue;
        
        double lat = min_lat + i * step_size_lat;
        std::vector<std::pair<double, double>> current_row_waypoints;
        
        if (going_right) {
            for (int j = 0; j < num_passes_lon; j++) {
                std::pair<double, double> wp = {lat, min_lon + j * step_size_lon};
                if (geometry_utils_->is_point_in_fence(wp, geofence_corners) &&
                    geometry_utils_->is_point_safe_distance(wp, geofence_corners, safe_dist_lat, safe_dist_lon)) {
                    current_row_waypoints.push_back(wp);
                }
            }
        } else {
            for (int j = num_passes_lon - 1; j >= 0; j--) {
                std::pair<double, double> wp = {lat, min_lon + j * step_size_lon};
                if (geometry_utils_->is_point_in_fence(wp, geofence_corners) &&
                    geometry_utils_->is_point_safe_distance(wp, geofence_corners, safe_dist_lat, safe_dist_lon)) {
                    current_row_waypoints.push_back(wp);
                }
            }
        }
        
        if (!current_row_waypoints.empty()) {
            for (const auto& wp_pair : current_row_waypoints) {
                sensor_msgs::msg::NavSatFix wp;
                wp.latitude = wp_pair.first;
                wp.longitude = wp_pair.second;
                wp.altitude = target_altitude;
                waypoints.push_back(wp);
            }
            row_processed[i] = true;
            going_right = !going_right;
        }
    }

    return waypoints;
}





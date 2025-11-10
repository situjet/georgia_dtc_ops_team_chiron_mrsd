#pragma once

#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

/**
 * @brief 状态机转换记录器
 * 
 * 记录所有状态机的转换，包括：
 * - Action的激活、成功、失败
 * - Condition的变化
 * - 相关的conditions状态
 * - 时间戳和详细信息
 * 
 * 输出到两个地方：
 * 1. ROS日志 (RCLCPP_INFO)
 * 2. CSV文件（便于后续分析）
 */
class TransitionLogger {
public:
    /**
     * @brief 构造函数
     * @param node ROS节点指针，用于日志输出
     * @param log_dir 日志文件目录（默认：~/.ros/transitions/）
     * @param log_to_console 是否输出到控制台
     * @param log_to_file 是否输出到文件
     */
    explicit TransitionLogger(
        rclcpp::Node* node,
        const std::string& log_dir = "",
        bool log_to_console = true,
        bool log_to_file = true);

    ~TransitionLogger();

    /**
     * @brief 记录Action转换
     * @param action_name Action名称
     * @param status 状态（ACTIVATED, RUNNING, SUCCESS, FAILURE）
     * @param conditions 相关conditions状态的map
     * @param failure_reason 失败原因（如果有）
     * @param additional_info 额外信息
     */
    void log_action_transition(
        const std::string& action_name,
        const std::string& status,
        const std::map<std::string, bool>& conditions,
        const std::string& failure_reason = "",
        const std::string& additional_info = "");

    /**
     * @brief 记录Condition变化
     * @param condition_name Condition名称
     * @param old_value 旧值
     * @param new_value 新值
     */
    void log_condition_change(
        const std::string& condition_name,
        bool old_value,
        bool new_value);

    /**
     * @brief 刷新缓冲区，确保数据写入文件
     */
    void flush();

    /**
     * @brief 获取日志文件路径
     */
    std::string get_log_file_path() const { return log_file_path_; }

private:
    /**
     * @brief 获取当前时间戳字符串
     * @return 格式化的时间戳（YYYY-MM-DD HH:MM:SS.mmm）
     */
    std::string get_timestamp() const;

    /**
     * @brief 将map转换为JSON格式字符串
     * @param conditions conditions的map
     * @return JSON格式字符串，如：{armed:false, landed:true}
     */
    std::string conditions_to_string(const std::map<std::string, bool>& conditions) const;

    /**
     * @brief 转义CSV字段中的特殊字符
     */
    std::string escape_csv_field(const std::string& field) const;

    /**
     * @brief 写入CSV头部
     */
    void write_csv_header();

    rclcpp::Node* node_;
    std::string log_file_path_;
    std::ofstream log_file_;
    std::mutex file_mutex_;
    
    bool log_to_console_;
    bool log_to_file_;
    bool file_initialized_;
};





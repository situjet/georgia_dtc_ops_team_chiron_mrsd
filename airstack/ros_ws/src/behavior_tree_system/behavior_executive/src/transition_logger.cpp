#include "transition_logger.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>

TransitionLogger::TransitionLogger(
    rclcpp::Node* node,
    const std::string& log_dir,
    bool log_to_console,
    bool log_to_file)
    : node_(node),
      log_to_console_(log_to_console),
      log_to_file_(log_to_file),
      file_initialized_(false) {
    
    if (log_to_file_) {
        // 确定日志目录
        std::string dir = log_dir;
        if (dir.empty()) {
            const char* home = getenv("HOME");
            if (home) {
                dir = std::string(home) + "/.ros/transitions/";
            } else {
                dir = "/tmp/transitions/";
            }
        }
        
        // 创建目录
        mkdir(dir.c_str(), 0755);
        
        // 生成带时间戳的文件名
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << dir << "transitions_" 
           << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
           << ".csv";
        
        log_file_path_ = ss.str();
        
        // 打开文件
        log_file_.open(log_file_path_, std::ios::out | std::ios::app);
        if (log_file_.is_open()) {
            write_csv_header();
            file_initialized_ = true;
            RCLCPP_INFO(node_->get_logger(), 
                       "[TransitionLogger] Logging transitions to: %s", 
                       log_file_path_.c_str());
        } else {
            RCLCPP_ERROR(node_->get_logger(), 
                        "[TransitionLogger] Failed to open log file: %s", 
                        log_file_path_.c_str());
        }
    }
}

TransitionLogger::~TransitionLogger() {
    if (log_file_.is_open()) {
        flush();
        log_file_.close();
    }
}

void TransitionLogger::log_action_transition(
    const std::string& action_name,
    const std::string& status,
    const std::map<std::string, bool>& conditions,
    const std::string& failure_reason,
    const std::string& additional_info) {
    
    std::string timestamp = get_timestamp();
    std::string conditions_str = conditions_to_string(conditions);
    
    // 输出到ROS日志
    if (log_to_console_) {
        std::stringstream log_msg;
        log_msg << "[TRANSITION] [" << timestamp << "] "
                << "ACTION: " << action_name << " | "
                << "STATUS: " << status << " | "
                << "CONDITIONS: " << conditions_str;
        
        if (!failure_reason.empty()) {
            log_msg << " | FAILURE: " << failure_reason;
        }
        
        if (!additional_info.empty()) {
            log_msg << " | INFO: " << additional_info;
        }
        
        RCLCPP_INFO(node_->get_logger(), "%s", log_msg.str().c_str());
    }
    
    // 写入CSV文件
    if (log_to_file_ && file_initialized_) {
        std::lock_guard<std::mutex> lock(file_mutex_);
        
        log_file_ << escape_csv_field(timestamp) << ","
                  << "ACTION" << ","
                  << escape_csv_field(action_name) << ","
                  << escape_csv_field(status) << ","
                  << escape_csv_field(conditions_str) << ","
                  << (status == "SUCCESS" || status == "RUNNING" ? "SUCCESS" : "PENDING") << ","
                  << escape_csv_field(failure_reason) << ","
                  << escape_csv_field(additional_info) << "\n";
        
        log_file_.flush();  // 立即刷新，确保数据写入
    }
}

void TransitionLogger::log_condition_change(
    const std::string& condition_name,
    bool old_value,
    bool new_value) {
    
    std::string timestamp = get_timestamp();
    
    // 输出到ROS日志
    if (log_to_console_) {
        RCLCPP_INFO(node_->get_logger(), 
                   "[TRANSITION] [%s] CONDITION: %s | CHANGED: %s -> %s",
                   timestamp.c_str(),
                   condition_name.c_str(),
                   old_value ? "true" : "false",
                   new_value ? "true" : "false");
    }
    
    // 写入CSV文件
    if (log_to_file_ && file_initialized_) {
        std::lock_guard<std::mutex> lock(file_mutex_);
        
        std::stringstream change_info;
        change_info << (old_value ? "true" : "false") 
                   << "->" 
                   << (new_value ? "true" : "false");
        
        log_file_ << escape_csv_field(timestamp) << ","
                  << "CONDITION" << ","
                  << escape_csv_field(condition_name) << ","
                  << "CHANGED" << ","
                  << "\"\"" << ","  // no conditions for condition changes
                  << "\"\"" << ","  // no result
                  << "\"\"" << ","  // no failure reason
                  << escape_csv_field(change_info.str()) << "\n";
        
        log_file_.flush();
    }
}

void TransitionLogger::flush() {
    if (log_file_.is_open()) {
        std::lock_guard<std::mutex> lock(file_mutex_);
        log_file_.flush();
    }
}

std::string TransitionLogger::get_timestamp() const {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S")
       << "." << std::setfill('0') << std::setw(3) << ms.count();
    
    return ss.str();
}

std::string TransitionLogger::conditions_to_string(
    const std::map<std::string, bool>& conditions) const {
    
    if (conditions.empty()) {
        return "{}";
    }
    
    std::stringstream ss;
    ss << "{";
    bool first = true;
    for (const auto& [name, value] : conditions) {
        if (!first) ss << ", ";
        ss << name << ":" << (value ? "true" : "false");
        first = false;
    }
    ss << "}";
    
    return ss.str();
}

std::string TransitionLogger::escape_csv_field(const std::string& field) const {
    // 如果字段包含逗号、引号或换行符，需要用引号包裹并转义
    if (field.find(',') != std::string::npos ||
        field.find('"') != std::string::npos ||
        field.find('\n') != std::string::npos) {
        
        std::string escaped = "\"";
        for (char c : field) {
            if (c == '"') {
                escaped += "\"\"";  // 双引号转义
            } else {
                escaped += c;
            }
        }
        escaped += "\"";
        return escaped;
    }
    
    return field;
}

void TransitionLogger::write_csv_header() {
    log_file_ << "timestamp,event_type,name,status,conditions,result,failure_reason,notes\n";
    log_file_.flush();
}





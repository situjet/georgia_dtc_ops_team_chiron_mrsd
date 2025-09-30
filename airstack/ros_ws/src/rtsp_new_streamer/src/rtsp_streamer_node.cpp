#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <memory>
#include <vector>
#include <chrono>

class RTSPStreamerNode : public rclcpp::Node
{
public:
    RTSPStreamerNode() : Node("rtsp_streamer_node")
    {
        // RTSP URLs
        declare_parameter("eo_rtsp_url", "rtsp://10.3.1.124:8556/ghadron");
        declare_parameter("ir_rtsp_url", "rtsp://10.3.1.124:8556/ghadron");
        
        // HQ (High Quality) topics and settings
        declare_parameter("eo_hq_topic", "/vehicle_9/eo/image/compressed");
        declare_parameter("ir_hq_topic", "/vehicle_9/ir/image/compressed");
        declare_parameter("hq_fps", 30);
        declare_parameter("hq_jpeg_quality", 90);
        
        // LQ (Low Quality) topics and settings
        declare_parameter("eo_lq_topic", "/vehicle_9/eo/image_lq/compressed");
        declare_parameter("ir_lq_topic", "/vehicle_9/ir/image_lq/compressed");
        declare_parameter("enable_lq", true);
        declare_parameter("lq_scale_factor", 0.5);  // 0.5 = half resolution
        declare_parameter("lq_fps_divisor", 2);      // 2 = half framerate
        declare_parameter("lq_jpeg_quality", 50);    
        declare_parameter("lq_target_bitrate_mbps", 2.0);  // Target bitrate in Mbps
        
        // General settings
        declare_parameter("enable_eo", true);
        declare_parameter("enable_ir", true);
        declare_parameter("reconnect_delay_sec", 5);
        
        // Get parameters
        eo_rtsp_url_ = get_parameter("eo_rtsp_url").as_string();
        ir_rtsp_url_ = get_parameter("ir_rtsp_url").as_string();
        
        eo_hq_topic_ = get_parameter("eo_hq_topic").as_string();
        ir_hq_topic_ = get_parameter("ir_hq_topic").as_string();
        eo_lq_topic_ = get_parameter("eo_lq_topic").as_string();
        ir_lq_topic_ = get_parameter("ir_lq_topic").as_string();
        
        enable_eo_ = get_parameter("enable_eo").as_bool();
        enable_ir_ = get_parameter("enable_ir").as_bool();
        enable_lq_ = get_parameter("enable_lq").as_bool();
        
        hq_fps_ = get_parameter("hq_fps").as_int();
        hq_jpeg_quality_ = get_parameter("hq_jpeg_quality").as_int();
        
        lq_scale_factor_ = get_parameter("lq_scale_factor").as_double();
        lq_fps_divisor_ = get_parameter("lq_fps_divisor").as_int();
        lq_jpeg_quality_ = get_parameter("lq_jpeg_quality").as_int();
        lq_target_bitrate_mbps_ = get_parameter("lq_target_bitrate_mbps").as_double();
        
        reconnect_delay_sec_ = get_parameter("reconnect_delay_sec").as_int();
        
        // Create publishers
        if (enable_eo_) {
            eo_hq_publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(eo_hq_topic_, 10);
            RCLCPP_INFO(get_logger(), "EO HQ stream enabled on topic: %s", eo_hq_topic_.c_str());
            
            if (enable_lq_) {
                eo_lq_publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(eo_lq_topic_, 10);
                RCLCPP_INFO(get_logger(), "EO LQ stream enabled on topic: %s (scale: %.2f, fps divisor: %d)",
                           eo_lq_topic_.c_str(), lq_scale_factor_, lq_fps_divisor_);
            }
            
            eo_thread_ = std::thread(&RTSPStreamerNode::eoStreamLoop, this);
        }
        
        if (enable_ir_) {
            ir_hq_publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(ir_hq_topic_, 10);
            RCLCPP_INFO(get_logger(), "IR HQ stream enabled on topic: %s", ir_hq_topic_.c_str());
            
            if (enable_lq_) {
                ir_lq_publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(ir_lq_topic_, 10);
                RCLCPP_INFO(get_logger(), "IR LQ stream enabled on topic: %s (scale: %.2f, fps divisor: %d)",
                           ir_lq_topic_.c_str(), lq_scale_factor_, lq_fps_divisor_);
            }
            
            ir_thread_ = std::thread(&RTSPStreamerNode::irStreamLoop, this);
        }
    }
    
    ~RTSPStreamerNode()
    {
        running_ = false;
        if (eo_thread_.joinable()) {
            eo_thread_.join();
        }
        if (ir_thread_.joinable()) {
            ir_thread_.join();
        }
    }
    
private:
    void eoStreamLoop()
    {
        streamLoop(eo_rtsp_url_, eo_hq_publisher_, eo_lq_publisher_, "EO");
    }
    
    void irStreamLoop()
    {
        streamLoop(ir_rtsp_url_, ir_hq_publisher_, ir_lq_publisher_, "IR");
    }
    
    int calculateJpegQuality(const cv::Mat& frame, double target_bitrate_mbps, int base_quality)
    {
        // Calculate approximate bytes per frame for target bitrate
        double target_bytes_per_frame = (target_bitrate_mbps * 1000000.0) / (8.0 * hq_fps_ / lq_fps_divisor_);
        
        // Start with base quality and adjust if needed
        int quality = base_quality;
        std::vector<uchar> buffer;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, quality};
        
        cv::imencode(".jpg", frame, buffer, params);
        
        // Simple adjustment: if too large, reduce quality
        if (buffer.size() > target_bytes_per_frame * 1.2) {
            quality = std::max(20, quality - 10);
        } else if (buffer.size() < target_bytes_per_frame * 0.8) {
            quality = std::min(95, quality + 5);
        }
        
        return quality;
    }
    
    void streamLoop(const std::string& rtsp_url, 
                   rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr hq_publisher,
                   rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr lq_publisher,
                   const std::string& stream_name)
    {
        uint64_t frame_count = 0;
        int adaptive_lq_quality = lq_jpeg_quality_;
        
        while (running_ && rclcpp::ok()) {
            cv::VideoCapture cap;
            
            RCLCPP_INFO(get_logger(), "Connecting to %s stream: %s", stream_name.c_str(), rtsp_url.c_str());
            
            cap.open(rtsp_url, cv::CAP_FFMPEG);
            cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
            cap.set(cv::CAP_PROP_FPS, hq_fps_);
            
            if (!cap.isOpened()) {
                RCLCPP_ERROR(get_logger(), "Failed to open %s stream: %s. Retrying in %d seconds...",
                           stream_name.c_str(), rtsp_url.c_str(), reconnect_delay_sec_);
                std::this_thread::sleep_for(std::chrono::seconds(reconnect_delay_sec_));
                continue;
            }
            
            RCLCPP_INFO(get_logger(), "%s stream connected successfully", stream_name.c_str());
            
            cv::Mat frame;
            while (running_ && rclcpp::ok()) {
                if (!cap.read(frame)) {
                    RCLCPP_WARN(get_logger(), "%s stream read failed, reconnecting...", stream_name.c_str());
                    break;
                }
                
                if (frame.empty()) {
                    continue;
                }
                
                auto timestamp = now();
                frame_count++;
                
                // Publish HQ stream (always)
                auto hq_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
                hq_msg->header.stamp = timestamp;
                hq_msg->header.frame_id = stream_name == "EO" ? "eo_camera" : "ir_camera";
                hq_msg->format = "jpeg";
                
                std::vector<int> hq_params = {cv::IMWRITE_JPEG_QUALITY, hq_jpeg_quality_};
                cv::imencode(".jpg", frame, hq_msg->data, hq_params);
                hq_publisher->publish(*hq_msg);
                
                // Publish LQ stream (if enabled and at reduced framerate)
                if (enable_lq_ && lq_publisher && (frame_count % lq_fps_divisor_ == 0)) {
                    auto lq_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
                    lq_msg->header.stamp = timestamp;
                    lq_msg->header.frame_id = (stream_name == "EO" ? "eo_camera_lq" : "ir_camera_lq");
                    lq_msg->format = "jpeg";
                    
                    // Resize frame for LQ stream
                    cv::Mat lq_frame;
                    if (std::abs(lq_scale_factor_ - 1.0) > 0.01) {
                        cv::resize(frame, lq_frame, cv::Size(), lq_scale_factor_, lq_scale_factor_, cv::INTER_AREA);
                    } else {
                        lq_frame = frame;
                    }
                    
                    // Calculate adaptive quality based on target bitrate
                    if (frame_count % (lq_fps_divisor_ * 10) == 0) {  // Adjust every 10 LQ frames
                        adaptive_lq_quality = calculateJpegQuality(lq_frame, lq_target_bitrate_mbps_, adaptive_lq_quality);
                    }
                    
                    std::vector<int> lq_params = {cv::IMWRITE_JPEG_QUALITY, adaptive_lq_quality};
                    cv::imencode(".jpg", lq_frame, lq_msg->data, lq_params);
                    
                    lq_publisher->publish(*lq_msg);
                    
                    // Log bitrate occasionally
                    if (frame_count % (lq_fps_divisor_ * 30) == 0) {
                        double actual_bitrate = (lq_msg->data.size() * 8.0 * hq_fps_ / lq_fps_divisor_) / 1000000.0;
                        RCLCPP_DEBUG(get_logger(), "%s LQ: size=%zu bytes, quality=%d, bitrate=%.2f Mbps",
                                    stream_name.c_str(), lq_msg->data.size(), adaptive_lq_quality, actual_bitrate);
                    }
                }
            }
            
            cap.release();
            
            if (running_ && rclcpp::ok()) {
                RCLCPP_INFO(get_logger(), "%s stream disconnected, reconnecting in %d seconds...",
                          stream_name.c_str(), reconnect_delay_sec_);
                std::this_thread::sleep_for(std::chrono::seconds(reconnect_delay_sec_));
            }
        }
    }
    
    // RTSP URLs
    std::string eo_rtsp_url_;
    std::string ir_rtsp_url_;
    
    // Topic names
    std::string eo_hq_topic_;
    std::string ir_hq_topic_;
    std::string eo_lq_topic_;
    std::string ir_lq_topic_;
    
    // Enable flags
    bool enable_eo_;
    bool enable_ir_;
    bool enable_lq_;
    
    // HQ settings
    int hq_fps_;
    int hq_jpeg_quality_;
    
    // LQ settings
    double lq_scale_factor_;
    int lq_fps_divisor_;
    int lq_jpeg_quality_;
    double lq_target_bitrate_mbps_;
    
    // General settings
    int reconnect_delay_sec_;
    
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr eo_hq_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr eo_lq_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr ir_hq_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr ir_lq_publisher_;
    
    // Threads
    std::thread eo_thread_;
    std::thread ir_thread_;
    std::atomic<bool> running_{true};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RTSPStreamerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
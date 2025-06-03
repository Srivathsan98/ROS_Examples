#ifndef ROS2_FDR_CPP_VIDEO_RECORDING_NODE_HPP_
#define ROS2_FDR_CPP_VIDEO_RECORDING_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <string>
#include <filesystem>
#include <mutex>
#include <queue>

namespace ros2_fdr_cpp {

class VideoRecordingNode : public rclcpp::Node {
public:
    VideoRecordingNode();
    ~VideoRecordingNode();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void startNewVideoFile();
    void stopCurrentVideoFile();
    std::string getCurrentTimestamp();
    std::string getCurrentDateFolder();
    void ensureDirectoryExists(const std::string& path);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    cv::VideoWriter video_writer_;
    std::string current_video_path_;
    std::chrono::steady_clock::time_point last_file_creation_time_;
    bool is_recording_;
    std::mutex recording_mutex_;
    
    // Configuration parameters
    const int RECORDING_DURATION_SECONDS = 120;  // 2 minutes per file
    const std::string RECORDINGS_DIR = "recordings";
};

} // namespace ros2_fdr_cpp

#endif // ROS2_FDR_CPP_VIDEO_RECORDING_NODE_HPP_ 
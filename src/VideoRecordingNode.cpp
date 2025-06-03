#include "VideoRecordingNode.hpp"
#include <rclcpp/rclcpp.hpp>
#include <filesystem>

namespace ros2_fdr_cpp {

VideoRecordingNode::VideoRecordingNode() 
    : Node("video_recording_node"), is_recording_(false) {
    
    // Create subscriptions
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "query_frames", 10,
        std::bind(&VideoRecordingNode::imageCallback, this, std::placeholders::_1));

    // Initialize recording state
    last_file_creation_time_ = std::chrono::steady_clock::now();
    is_recording_ = true;  // Start recording immediately
    
    RCLCPP_INFO(this->get_logger(), "Video Recording Node initialized");
}

VideoRecordingNode::~VideoRecordingNode() {
    // Stop recording and close the video writer
    is_recording_ = false;
    stopCurrentVideoFile();
    RCLCPP_INFO(this->get_logger(), "Video Recording Node shutdown complete");
}

void VideoRecordingNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(recording_mutex_);
    
    if (!is_recording_) return;

    try {
        // Convert ROS image message to OpenCV format
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat frame = cv_ptr->image;

        // If this is the first frame or we need a new file, get the frame dimensions
        if (!video_writer_.isOpened()) {
            int frame_width = frame.cols;
            int frame_height = frame.rows;
            double fps = 30.0;

            // Create the video writer with the correct dimensions
            std::string date_folder = getCurrentDateFolder();
            ensureDirectoryExists(date_folder);

            std::string timestamp = getCurrentTimestamp();
            current_video_path_ = date_folder + "/recording_" + timestamp + ".mp4";

            video_writer_.open(current_video_path_, 
                            cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                            fps, cv::Size(frame_width, frame_height));

            if (!video_writer_.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open video writer for: %s", current_video_path_.c_str());
                return;
            }

            last_file_creation_time_ = std::chrono::steady_clock::now();
            RCLCPP_INFO(this->get_logger(), "Started new video file: %s", current_video_path_.c_str());
        }

        // Check if we need to create a new video file
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            current_time - last_file_creation_time_).count();

        if (elapsed >= RECORDING_DURATION_SECONDS) {
            stopCurrentVideoFile();
            // The next frame will trigger creation of a new file
        }

        // Write frame to video file
        if (video_writer_.isOpened()) {
            video_writer_.write(frame);
            RCLCPP_DEBUG(this->get_logger(), "Wrote frame of size %dx%d", frame.cols, frame.rows);
        }

    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void VideoRecordingNode::startNewVideoFile() {
    // This function is now handled in imageCallback
}

void VideoRecordingNode::stopCurrentVideoFile() {
    if (video_writer_.isOpened()) {
        video_writer_.release();
        RCLCPP_INFO(this->get_logger(), "Closed video file: %s", current_video_path_.c_str());
    }
}

std::string VideoRecordingNode::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S");
    ss << '_' << std::setfill('0') << std::setw(3) << now_ms.count();
    return ss.str();
}

std::string VideoRecordingNode::getCurrentDateFolder() {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d");
    return RECORDINGS_DIR + "/" + ss.str();
}

void VideoRecordingNode::ensureDirectoryExists(const std::string& path) {
    std::filesystem::create_directories(path);
}

} // namespace ros2_fdr_cpp 
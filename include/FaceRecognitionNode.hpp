#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "framecapture.h"
#include "facedetector.h"
#include "facerecognizer.h"
#include "config.h"
#include <mutex>

class FaceRecognitionNode : public rclcpp::Node {
public:
    FaceRecognitionNode();
    ~FaceRecognitionNode();

    // Method to get the last processed query frame (thread-safe)
    cv::Mat getLastQueryFrame() const {
        std::lock_guard<std::mutex> lock(m_frameMutex);
        return m_lastQueryFrame.clone();
    }

private:
    void processFrame();
    void publishQueryFrame(const cv::Mat& frame);
    std::string getCurrentDateFolder();
    void ensureDirectoryExists(const std::string& path);
    
    // Frame processing components
    FrameCapture m_frameCapture;
    std::unique_ptr<FaceDetector> m_faceDetector;
    std::unique_ptr<FaceRecognizer> m_faceRecognizer;
    
    // ROS2 components
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_queryFramePublisher;
    std::atomic<bool> m_running{true};
    std::thread m_captureThread;
    
    // Video components
    cv::VideoWriter m_gstWriter;
    
    // State
    cv::Mat m_targetImage;
    cv::Mat m_targetFace;
    cv::Mat m_targetFeatures;
    mutable std::mutex m_frameMutex;
    cv::Mat m_lastQueryFrame;

    std::vector<cv::Mat> m_unrecognizedFaceCache;
    const double UNRECOGNIZED_SIM_THRESHOLD = 0.91;  // Tune this

    const std::string UNRECOG_DIR = "unrecognized_faces";
};

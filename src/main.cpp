#include "FaceRecognitionNode.hpp"
#include "VideoRecordingNode.hpp"
#include "config.h"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <signal.h>

// Define the global variables
std::string facedetection_modelpath = "/home/pvsp/ros2_ws/src/ros2_fdr_cpp/models/face_detection_model.onnx";
std::string facerecognition_modelpath = "/home/pvsp/ros2_ws/src/ros2_fdr_cpp/models/face_recognition_model.onnx";
std::string target_image_path = "/home/pvsp/ros2_ws/src/ros2_fdr_cpp/models/output_image.jpg";
std::string target_image_name = "target.jpg";

// Global variables for cleanup
std::shared_ptr<FaceRecognitionNode> face_recognition_node;
std::shared_ptr<ros2_fdr_cpp::VideoRecordingNode> video_recording_node;
rclcpp::executors::MultiThreadedExecutor* executor = nullptr;

void signalHandler(int signum) {
    if (executor) {
        RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down...");
        executor->cancel();
        executor->remove_node(face_recognition_node);
        executor->remove_node(video_recording_node);
        face_recognition_node->~FaceRecognitionNode();
        video_recording_node->~VideoRecordingNode();    
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // Set up signal handling
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try {
        // Create executor
        rclcpp::executors::MultiThreadedExecutor local_executor;
        executor = &local_executor;
        
        // Create nodes
        face_recognition_node = std::make_shared<FaceRecognitionNode>();
        video_recording_node = std::make_shared<ros2_fdr_cpp::VideoRecordingNode>();
        
        // Add nodes to executor
        executor->add_node(face_recognition_node);
        executor->add_node(video_recording_node);
        
        // Spin the executor
        executor->spin();
        
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Exception in main: %s", e.what());
        return 1;
    }
    
    // Cleanup
    face_recognition_node.reset();
    video_recording_node.reset();
    executor = nullptr;
    
    rclcpp::shutdown();
    return 0;
}
#include "FaceRecognitionNode.hpp"
#include "config.h"
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>

FaceRecognitionNode::FaceRecognitionNode()
    : Node("face_recognition_node") {
    
    // Initialize the publisher
    m_queryFramePublisher = this->create_publisher<sensor_msgs::msg::Image>(
        "query_frames", 10);

    // Initialize frame capture
    m_captureThread = std::thread(&FrameCapture::captureLoop, &m_frameCapture, std::ref(m_running));
    pthread_setname_np(m_captureThread.native_handle(), "CaptureLoop");

    // Initialize face detection and recognition
    m_faceDetector = std::make_unique<FaceDetector>(facedetection_modelpath, cv::Size(320, 320), 0.9f, 0.3f, 100, 0, 0);
    RCLCPP_INFO(this->get_logger(), "Face detection loaded");
    
    m_faceRecognizer = std::make_unique<FaceRecognizer>(facerecognition_modelpath, 0, 0, 0);
    RCLCPP_INFO(this->get_logger(), "Face recognition loaded");

    // Load target image
    if (target_image_path.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Target image path is empty");
        throw std::runtime_error("Target image path is empty");
    }

    m_targetImage = cv::imread(target_image_path);
    if (m_targetImage.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load target image from: %s", target_image_path.c_str());
        throw std::runtime_error("Failed to load target image");
    }

    // Process target image
    m_faceDetector->setFrameInputSize(m_targetImage.size());
    m_faceDetector->setdetectionTopK(3);
    m_targetFace = m_faceDetector->infer(m_targetImage);
    m_targetFeatures = m_faceRecognizer->extractfeatures(m_targetImage, m_targetFace);

    // Set up camera size
    const auto [w, h] = m_frameCapture.getCameraSize();
    m_faceDetector->setFrameInputSize(cv::Size(w, h));

    // Initialize GStreamer writer
    std::ostringstream pipeline;
    pipeline << "appsrc is-live=true block=true format=3 caps=video/x-raw,format=BGR,width=" << w * 2
             << ",height=" << h << ",framerate=30/1 ! "
             << "videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=500 ! "
             << "rtph264pay config-interval=1 pt=96 ! "
             << "udpsink host=127.0.0.1 port=5000";

    m_gstWriter.open(pipeline.str(), cv::CAP_GSTREAMER, 30.0, cv::Size(w * 2, h), true);
    if (!m_gstWriter.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open GStreamer pipeline");
        throw std::runtime_error("Failed to open GStreamer pipeline");
    }

    // Create timer for processing frames
    m_timer = this->create_wall_timer(
        std::chrono::milliseconds(33),  // ~30 FPS
        [this]() { this->processFrame(); });

    RCLCPP_INFO(this->get_logger(), "Face Recognition Node initialized");
}

FaceRecognitionNode::~FaceRecognitionNode() {
    m_running = false;
    if (m_captureThread.joinable()) {
        m_captureThread.join();
    }
    cv::destroyAllWindows();
}

void FaceRecognitionNode::processFrame() {
    while (m_running) {
        cv::Mat frame = m_frameCapture.getLatestFrame();
        if (frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        cv::Mat query_faces = m_faceDetector->infer(frame);
        if (query_faces.empty()) {
            continue;
        }

        // Process detected faces
        for (int i = 0; i < query_faces.rows; ++i) {
            int x1 = static_cast<int>(query_faces.at<float>(i, 0));
            int y1 = static_cast<int>(query_faces.at<float>(i, 1));
            int w = static_cast<int>(query_faces.at<float>(i, 2));
            int h = static_cast<int>(query_faces.at<float>(i, 3));
            float conf = query_faces.at<float>(i, 14);
        }

        // Extract and match features
        cv::Mat query_features = m_faceRecognizer->extractfeatures(frame, query_faces.row(0));
        const auto match = m_faceRecognizer->matchFeatures(m_targetFeatures, query_features);

        // Create visualization
        auto vis_target = m_frameCapture.visualize(m_targetImage, m_targetFace, {{1.0, true}}, -0.1f, frame.size());
        auto vis_query = m_frameCapture.visualize(frame, query_faces, {match}, 30.00);
        
        cv::Mat output_image;
        cv::hconcat(vis_target, vis_query, output_image);

        if (output_image.empty()) {
            RCLCPP_WARN(this->get_logger(), "Output image is empty, skipping write");
            continue;
        }

        if (output_image.channels() != 3) {
            cv::cvtColor(output_image, output_image, cv::COLOR_GRAY2BGR);
        }

        // Store and publish the query frame
        {
            std::lock_guard<std::mutex> lock(m_frameMutex);
            m_lastQueryFrame = vis_query.clone();
        }
        publishQueryFrame(output_image);

        // Write to GStreamer for Qt application streaming
        m_gstWriter.write(output_image);

        // Handle keyboard input
        handleKeyboardInput();
    }
}

void FaceRecognitionNode::publishQueryFrame(const cv::Mat& frame) {
    try {
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        m_queryFramePublisher->publish(*msg);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void FaceRecognitionNode::handleKeyboardInput() {
    cv::waitKey(1);
}

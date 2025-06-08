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
    RCLCPP_INFO(this->get_logger(), "Target image size: %dx%d", m_targetImage.cols, m_targetImage.rows);

    // Process target image
    m_faceDetector->setFrameInputSize(m_targetImage.size());
    m_faceDetector->setdetectionTopK(1);
    m_targetFace = m_faceDetector->infer(m_targetImage);
    m_targetFeatures = m_faceRecognizer->extractfeatures(m_targetImage, m_targetFace.row(0));

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
    cv::Mat output_image;
    while (m_running) {
        cv::Mat frame = m_frameCapture.getLatestFrame();
        if (frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        m_faceDetector->setFrameInputSize(frame.size());
        m_faceDetector->setdetectionTopK(5000);
        // RCLCPP_INFO(this->get_logger(), "Processing frame of size: %dx%d", frame.cols, frame.rows);

        cv::Mat query_faces = m_faceDetector->infer(frame);
        // RCLCPP_INFO(this->get_logger(), "Detected %d faces", query_faces.rows);

        if (query_faces.empty()) {
            // continue;
            output_image = frame.clone();
            // Store and publish the query frame
        {
            std::lock_guard<std::mutex> lock(m_frameMutex);
            m_lastQueryFrame = output_image.clone();
        }
        }

        std::vector<std::pair<double, bool>> matches;

        if(!query_faces.empty())
        {

        // Process detected faces
        for (int i = 0; i < query_faces.rows; ++i) {
            cv::Mat query_features = m_faceRecognizer->extractfeatures(frame, query_faces.row(i));
            RCLCPP_INFO(this->get_logger(), "Matched %zu faces", matches.size());
            // Measure similarity of target face to query face
            const auto match = m_faceRecognizer->matchFeatures(m_targetFeatures, query_features);
            matches.push_back(match);
            int x1 = static_cast<int>(query_faces.at<float>(i, 0));
            int y1 = static_cast<int>(query_faces.at<float>(i, 1));
            int w = static_cast<int>(query_faces.at<float>(i, 2));
            int h = static_cast<int>(query_faces.at<float>(i, 3));
            float conf = query_faces.at<float>(i, 14);
        }

        // Extract and match features
        // cv::Mat query_features = m_faceRecognizer->extractfeatures(frame, query_faces);
        // // std::vector<std::pair<double, bool>> match_results = m_faceRecognizer->matchFeatures(m_targetFeatures, query_features);
        // RCLCPP_INFO(this->get_logger(), "Matched %zu faces", match_results.size());

        // const auto match = m_faceRecognizer->matchFeatures(m_targetFeatures, query_features);

        // Create visualization
        auto vis_target = m_frameCapture.visualize(m_targetImage, m_targetFace, {{1.0, true}}, -0.1f, frame.size());
        auto vis_query = m_frameCapture.visualize(frame, query_faces, matches, 30.00);
/*************************************************************************************************************************** */
        //  std::vector<std::pair<double, bool>> match_results;


        // if (!query_faces.empty()) {
        //     for (int i = 0; i < query_faces.rows; ++i) {
        //         cv::Mat single_face = query_faces.row(i);
        //         cv::Mat query_features = m_faceRecognizer->extractfeatures(frame, single_face);

        //         if (query_features.empty()) {
        //             match_results.emplace_back(0.0f, false);
        //             continue;
        //         }

        //         auto match = m_faceRecognizer->matchFeatures(m_targetFeatures, query_features);
        //         match_results.push_back(match);
        //     }
        // }

        // // Create visualizations
        // auto vis_target = m_frameCapture.visualize(
        //     m_targetImage, m_targetFace, {{1.0f, true}}, -0.1f, frame.size());

        // auto vis_query = m_frameCapture.visualize(
        //     frame, query_faces, match_results, 30.0f);
/**************************************************************************************************************************************** */
        
        cv::hconcat(vis_target, vis_query, output_image);
        {
            std::lock_guard<std::mutex> lock(m_frameMutex);
            m_lastQueryFrame = vis_query.clone();
        }
        }

        if (output_image.empty()) {
            RCLCPP_WARN(this->get_logger(), "Output image is empty, skipping write");
            continue;
        }

        // output_image = query_faces.clone();
        if (output_image.channels() != 3) {
            cv::cvtColor(output_image, output_image, cv::COLOR_GRAY2BGR);
        }

        
        publishQueryFrame(output_image);

        // Write to GStreamer for Qt application streaming
        m_gstWriter.write(output_image);

        // Handle keyboard input
        // handleKeyboardInput();
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

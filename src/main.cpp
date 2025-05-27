// File: main.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <sstream>

#include "framecapture.h"
#include "facedetector.h"
#include "facerecognizer.h"
#include "config.h"


class FaceRecognitionNode : public rclcpp::Node {
public:
    FaceRecognitionNode()
        : Node("face_recognition_node"),
          m_frameCapture(),
          running_(true)
    {
        // Start capture thread
        captureThread_ = std::thread(&FrameCapture::captureLoop, &m_frameCapture, std::ref(running_));
        pthread_setname_np(captureThread_.native_handle(), "CaptureLoop");

        // Initialize face detection and recognition
        facedetector_ = std::make_unique<FaceDetector>(facedetection_modelpath, cv::Size(320, 320), 0.9f, 0.3f, 100, 0, 0);
        RCLCPP_INFO(this->get_logger(), "Face detection loaded");
        
        face_recognizer_ = std::make_unique<FaceRecognizer>(facerecognition_modelpath, 0, 0, 0);
        RCLCPP_INFO(this->get_logger(), "Face recognition loaded");

        // Load target image
        if(target_image_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Target image path is empty");
            throw std::runtime_error("Target image path is empty");
        }

        target_image_ = cv::imread(target_image_path);
        if (target_image_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load target image from: %s", target_image_path.c_str());
            throw std::runtime_error("Failed to load target image");
        }

        // Process target image
        facedetector_->setFrameInputSize(target_image_.size());
        facedetector_->setdetectionTopK(3);
        target_face_ = facedetector_->infer(target_image_);
        target_features_ = face_recognizer_->extractfeatures(target_image_, target_face_);

        // Set up camera size
        const auto [w, h] = m_frameCapture.getCameraSize();
        facedetector_->setFrameInputSize(cv::Size(w, h));

        // Initialize GStreamer writer
        std::ostringstream pipeline;
        pipeline << "appsrc is-live=true block=true format=3 caps=video/x-raw,format=BGR,width=" << w * 2
                 << ",height=" << h << ",framerate=30/1 ! "
                 << "videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=500 ! "
                 << "rtph264pay config-interval=1 pt=96 ! "
                 << "udpsink host=127.0.0.1 port=5000";

        gst_writer_.open(pipeline.str(), cv::CAP_GSTREAMER, 30.0, cv::Size(w * 2, h), true);
        if (!gst_writer_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open GStreamer pipeline");
            throw std::runtime_error("Failed to open GStreamer pipeline");
        }

        // Create timer for processing frames
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),  // ~30 FPS
            std::bind(&FaceRecognitionNode::processFrame, this));

        RCLCPP_INFO(this->get_logger(), "Face Recognition Node initialized");
    }

    ~FaceRecognitionNode() {
        running_ = false;
        if (captureThread_.joinable()) {
            captureThread_.join();
        }
        cv::destroyAllWindows();
    }

private:
    void processFrame() {
        cv::Mat query_frame = m_frameCapture.getLatestFrame();
        if (query_frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame captured, skipping...");
            return;
        }

        cv::Mat query_faces = facedetector_->infer(query_frame);
        if (query_faces.empty()) {
            return;
        }

        // Process detected faces
        for(int i = 0; i < query_faces.rows; ++i) {
            int x1 = static_cast<int>(query_faces.at<float>(i, 0));
            int y1 = static_cast<int>(query_faces.at<float>(i, 1));
            int w = static_cast<int>(query_faces.at<float>(i, 2));
            int h = static_cast<int>(query_faces.at<float>(i, 3));
            float conf = query_faces.at<float>(i, 14);
        }

        // Extract and match features
        cv::Mat query_features = face_recognizer_->extractfeatures(query_frame, query_faces.row(0));
        const auto match = face_recognizer_->matchFeatures(target_features_, query_features);

        // Create visualization
        auto vis_target = m_frameCapture.visualize(target_image_, target_face_, {{1.0, true}}, -0.1f, query_frame.size());
        auto vis_query = m_frameCapture.visualize(query_frame, query_faces, {match}, 30.00);
        
        cv::Mat output_image;
        cv::hconcat(vis_target, vis_query, output_image);

        if(output_image.empty()) {
            RCLCPP_WARN(this->get_logger(), "Output image is empty, skipping write");
            return;
        }

        if (output_image.channels() != 3) {
            cv::cvtColor(output_image, output_image, cv::COLOR_GRAY2BGR);
        }

        gst_writer_.write(output_image);
    }

private:
    FrameCapture m_frameCapture;
    std::unique_ptr<FaceDetector> facedetector_;
    std::unique_ptr<FaceRecognizer> face_recognizer_;
    cv::Mat target_image_;
    cv::Mat target_face_;
    cv::Mat target_features_;
    cv::VideoWriter gst_writer_;
    std::atomic<bool> running_;
    std::thread captureThread_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FaceRecognitionNode>());
    rclcpp::shutdown();
    return 0;
}
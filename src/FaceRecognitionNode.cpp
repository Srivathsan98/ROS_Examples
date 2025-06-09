#include "FaceRecognitionNode.hpp"
#include "config.h"
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <filesystem>
FaceRecognitionNode::FaceRecognitionNode()
    : Node("face_recognition_node")
{

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

    std::string date_folder = getCurrentDateFolder();
    ensureDirectoryExists(date_folder);

    // // Load target image
    // if (target_image_path.empty())
    // {
    //     RCLCPP_ERROR(this->get_logger(), "Target image path is empty");
    //     throw std::runtime_error("Target image path is empty");
    // }

    // m_targetImage = cv::imread(target_image_path);
    // if (m_targetImage.empty())
    // {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to load target image from: %s", target_image_path.c_str());
    //     throw std::runtime_error("Failed to load target image");
    // }
    // RCLCPP_INFO(this->get_logger(), "Target image size: %dx%d", m_targetImage.cols, m_targetImage.rows);

    // // Process target image
    // m_faceDetector->setFrameInputSize(m_targetImage.size());
    // m_faceDetector->setdetectionTopK(1);
    // m_targetFace = m_faceDetector->infer(m_targetImage);
    // m_targetFeatures = m_faceRecognizer->extractfeatures(m_targetImage, m_targetFace.row(0));

    for (const auto &path : target_image_paths)
    {
        cv::Mat targetImage = cv::imread(path);
        if (targetImage.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load target image from: %s", path.c_str());
            continue;
        }

        m_targetImages.push_back(targetImage);

        m_faceDetector->setFrameInputSize(targetImage.size());
        m_faceDetector->setdetectionTopK(1);

        cv::Mat targetFace = m_faceDetector->infer(targetImage);
        if (targetFace.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No face detected in target image: %s", path.c_str());
            continue;
        }

        m_targetFaces.push_back(targetFace);
        m_targetFeaturesList.push_back(
            m_faceRecognizer->extractfeatures(targetImage, targetFace.row(0)));

        RCLCPP_INFO(this->get_logger(), "Loaded target face from: %s", path.c_str());
    }

    // Set up camera size
    const auto [w, h] = m_frameCapture.getCameraSize();
    m_faceDetector->setFrameInputSize(cv::Size(w, h));

    // Initialize GStreamer writer
    std::ostringstream pipeline;
    pipeline << "appsrc is-live=true block=true format=3 caps=video/x-raw,format=BGR,width=" << w
             << ",height=" << h << ",framerate=30/1 ! "
             << "videoconvert ! x264enc tune=zerolatency speed-preset=superfast bitrate=2000 ! "
             << "rtph264pay config-interval=1 pt=96 ! "
             << "udpsink host=127.0.0.1 port=5000";

    m_gstWriter.open(pipeline.str(), cv::CAP_GSTREAMER, 30.0, cv::Size(w, h), true);
    if (!m_gstWriter.isOpened())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open GStreamer pipeline");
        throw std::runtime_error("Failed to open GStreamer pipeline");
    }

    // Create timer for processing frames
    m_timer = this->create_wall_timer(
        std::chrono::milliseconds(33), // ~30 FPS
        [this]()
        { this->processFrame(); });

    RCLCPP_INFO(this->get_logger(), "Face Recognition Node initialized");
}

FaceRecognitionNode::~FaceRecognitionNode()
{
    m_running = false;
    if (m_captureThread.joinable())
    {
        m_captureThread.join();
    }
    cv::destroyAllWindows();
}

void FaceRecognitionNode::processFrame()
{
    cv::Mat output_image;
    while (m_running)
    {
        cv::Mat frame = m_frameCapture.getLatestFrame();
        if (frame.empty())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        m_faceDetector->setFrameInputSize(frame.size());
        m_faceDetector->setdetectionTopK(5000);
        // RCLCPP_INFO(this->get_logger(), "Processing frame of size: %dx%d", frame.cols, frame.rows);

        cv::Mat query_faces = m_faceDetector->infer(frame);
        // RCLCPP_INFO(this->get_logger(), "Detected %d faces", query_faces.rows);

        std::vector<std::pair<double, bool>> matches;

        if (!query_faces.empty())
        {

            // Process detected faces
            for (int i = 0; i < query_faces.rows; ++i)
            {
                cv::Mat query_features = m_faceRecognizer->extractfeatures(frame, query_faces.row(i));
                //         if (query_features.empty()) {
                //     matches.emplace_back(0.0, false);  // Avoid crash, still show box
                //     continue;
                // }
                RCLCPP_INFO(this->get_logger(), "Matched %zu faces", matches.size());
                // Measure similarity of target face to query face
                // const auto match = m_faceRecognizer->matchFeatures(m_targetFeatures, query_features);
                // matches.push_back(match);

                double best_similarity = 0.0;
                bool matched = false;

                // for (const auto &target_features : m_targetFeaturesList)
                // {
                //     auto match = m_faceRecognizer->matchFeatures(target_features, query_features);
                //     if (match.second && match.first > best_similarity)
                //     {
                //         best_similarity = match.first;
                //         matched = true;
                //     }
                // }

                int matched_index = -1;

for (size_t t = 0; t < m_targetFeaturesList.size(); ++t)
{
    auto match = m_faceRecognizer->matchFeatures(m_targetFeaturesList[t], query_features);
    if (match.second && match.first > best_similarity)
    {
        best_similarity = match.first;
        matched = true;
        matched_index = static_cast<int>(t);
    }
}

                matches.emplace_back(best_similarity, matched);

                // If unrecognized
                // if (!match.second)
                if (!matched)
                {
                    bool is_new = true;
                    for (const auto &cached : m_unrecognizedFaceCache)
                    {
                        double sim = m_faceRecognizer->matchFeatures(cached, query_features).first;
                        if (sim > UNRECOGNIZED_SIM_THRESHOLD)
                        {
                            is_new = false;
                            break;
                        }
                    }

                    if (is_new)
                    {
                        m_unrecognizedFaceCache.push_back(query_features.clone());
                        // Save face region
                        int x = static_cast<int>(query_faces.at<float>(i, 0));
                        int y = static_cast<int>(query_faces.at<float>(i, 1));
                        int w = static_cast<int>(query_faces.at<float>(i, 2));
                        int h = static_cast<int>(query_faces.at<float>(i, 3));

                        // Clamp values to stay inside image bounds
                        x = std::max(0, x);
                        y = std::max(0, y);
                        w = std::min(w, frame.cols - x);
                        h = std::min(h, frame.rows - y);

                        // Only save if the bounding box is valid
                        if (w > 0 && h > 0 && x + w <= frame.cols && y + h <= frame.rows)
                        {
                            cv::Mat unrecognized_face = frame(cv::Rect(x, y, w, h)).clone();
                            std::string filename = UNRECOG_DIR + "/unrecognized_" + std::to_string(std::time(nullptr)) + ".jpg";
                            cv::imwrite(filename, unrecognized_face);
                        }
                        else
                        {
                            RCLCPP_WARN(this->get_logger(), "Invalid ROI, skipping unrecognized face save. x=%d y=%d w=%d h=%d", x, y, w, h);
                        }
                    }
                }
            }
            // Create visualization
            // auto vis_target = m_frameCapture.visualize(m_targetImage, m_targetFace, {{1.0, true}}, -0.1f, frame.size());
            RCLCPP_INFO(this->get_logger(), "Detected %d faces, Generated %zu matches", query_faces.rows, matches.size());

            auto vis_query = m_frameCapture.visualize(frame, query_faces, matches, 30.0f);
            output_image = vis_query.clone();
            {
                std::lock_guard<std::mutex> lock(m_frameMutex);
                m_lastQueryFrame = vis_query.clone();
            }
        }
        else
        {
            output_image = frame.clone();
            {
                std::lock_guard<std::mutex> lock(m_frameMutex);
                m_lastQueryFrame = output_image.clone();
            }
            RCLCPP_INFO(this->get_logger(), "No faces detected in the current frame");
        }

        if (output_image.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Output image is empty, skipping write");
            continue;
        }

        // output_image = query_faces.clone();
        if (output_image.channels() != 3)
        {
            cv::cvtColor(output_image, output_image, cv::COLOR_GRAY2BGR);
        }

        publishQueryFrame(output_image);

        // Write to GStreamer for Qt application streaming
        m_gstWriter.write(output_image);
    }
}

void FaceRecognitionNode::publishQueryFrame(const cv::Mat &frame)
{
    try
    {
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        m_queryFramePublisher->publish(*msg);
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

std::string FaceRecognitionNode::getCurrentDateFolder()
{
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d");
    return UNRECOG_DIR + "/" + ss.str();
}

void FaceRecognitionNode::ensureDirectoryExists(const std::string &path)
{
    std::filesystem::create_directories(path);
}
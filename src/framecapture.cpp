#include "framecapture.h"

FrameCapture::FrameCapture() {
    cap.open(0);
}

void FrameCapture::captureLoop(std::atomic<bool>& running) {
    cv::Mat frame;

    if(!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return;
    }
    while (running.load()) {
        cap >> frame;
        if (frame.empty()) continue;
        std::lock_guard<std::mutex> lock(mtx);
        frame.copyTo(latestFrame);

        std::this_thread::sleep_for(std::chrono::milliseconds(30)); // Adjust sleep duration as needed
        // sleep_for(std::chrono::milliseconds(30)); // Adjust sleep duration as needed
    }
}

cv::Mat FrameCapture::getLatestFrame() {
    std::lock_guard<std::mutex> lock(mtx);
    return latestFrame.clone();
}

std::pair <int, int> FrameCapture::getFrameSize(const cv::Mat& frame) {
    if (frame.empty()) {
        return {0, 0};
    }
    return {frame.cols, frame.rows};
}

std::pair <int, int> FrameCapture::getCameraSize() {
    if (!cap.isOpened()) {
        return {0, 0};
    }
    int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    return {width, height};
}
/*OepnCV Zoo*/
// cv::Mat FrameCapture::visualize(const cv::Mat& image,
//                   const cv::Mat& faces,
//                   const std::vector<std::pair<double, bool>>& matches,
//                   const float fps,
//                   const cv::Size& target_size)
// {
//     static const cv::Scalar matched_box_color{0, 255, 0};
//     static const cv::Scalar mismatched_box_color{0, 0, 255};

//     if (fps >= 0)
//     {
//         cv::Mat output_image = image.clone();

//         const int x1 = static_cast<int>(faces.at<float>(0, 0));
//         const int y1 = static_cast<int>(faces.at<float>(0, 1));
//         const int w = static_cast<int>(faces.at<float>(0, 2));
//         const int h = static_cast<int>(faces.at<float>(0, 3));
//         const auto match = matches.at(0);

//         cv::Scalar box_color = match.second ? matched_box_color : mismatched_box_color;
//         // Draw bounding box
//         cv::rectangle(output_image, cv::Rect(x1, y1, w, h), box_color, 2);
//         // Draw match score
//         cv::putText(output_image, cv::format("%.4f", match.first), cv::Point(x1, y1+12), cv::FONT_HERSHEY_DUPLEX, 0.30, box_color);
//         // Draw FPS
//         cv::putText(output_image, cv::format("FPS: %.2f", fps), cv::Point(0, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2);

//         return output_image;
//     }

//     cv::Mat output_image = cv::Mat::zeros(target_size, CV_8UC3);

//     // Determine new height and width of image with aspect ratio of original image
//     const double ratio = std::min(static_cast<double>(target_size.height) / image.rows,
//                                   static_cast<double>(target_size.width) / image.cols);
//     const int new_height = static_cast<int>(image.rows * ratio);
//     const int new_width = static_cast<int>(image.cols * ratio);

//     // Resize the original image, maintaining aspect ratio
//     cv::Mat resize_out;
//     cv::resize(image, resize_out, cv::Size(new_width, new_height), cv::INTER_LINEAR);

//     // Determine top left corner in resized dimensions
//     const int top = std::max(0, target_size.height - new_height) / 2;
//     const int left = std::max(0, target_size.width - new_width) / 2;

//     // Copy resized image into target output image
//     const cv::Rect roi = cv::Rect(cv::Point(left, top), cv::Size(new_width, new_height));
//     cv::Mat out_sub_image = output_image(roi);
//     resize_out.copyTo(out_sub_image);

//     for (int i = 0; i < faces.rows; ++i)
//     {
//         const int x1 = static_cast<int>(faces.at<float>(i, 0) * ratio) + left;
//         const int y1 = static_cast<int>(faces.at<float>(i, 1) * ratio) + top;
//         const int w = static_cast<int>(faces.at<float>(i, 2) * ratio);
//         const int h = static_cast<int>(faces.at<float>(i, 3) * ratio);
//         const auto match = matches.at(i);

//         cv::Scalar box_color = match.second ? matched_box_color : mismatched_box_color;
//         // Draw bounding box
//         cv::rectangle(output_image, cv::Rect(x1, y1, w, h), box_color, 2);
//         // Draw match score
//         cv::putText(output_image, cv::format("%.4f", match.first), cv::Point(x1, y1+12), cv::FONT_HERSHEY_DUPLEX, 0.30, box_color);
//     }
//     return output_image;
// }
/*********************************************************************************** */
cv::Mat FrameCapture::visualize(const cv::Mat& image,
                                const cv::Mat& faces,
                                const std::vector<std::pair<double, bool>>& matches,
                                const float fps,
                                const cv::Size& target_size)
{
    static const cv::Scalar matched_box_color{0, 255, 0};
    static const cv::Scalar mismatched_box_color{0, 0, 255};

    cv::Mat output_image = image.clone();

    // Draw all bounding boxes
    for (int i = 0; i < faces.rows; ++i)
    {
        const int x1 = static_cast<int>(faces.at<float>(i, 0));
        const int y1 = static_cast<int>(faces.at<float>(i, 1));
        const int w = static_cast<int>(faces.at<float>(i, 2));
        const int h = static_cast<int>(faces.at<float>(i, 3));

        const auto& match = matches.at(i);
        cv::Scalar box_color = match.second ? matched_box_color : mismatched_box_color;

        // Draw bounding box
        cv::rectangle(output_image, cv::Rect(x1, y1, w, h), box_color, 2);

        // Draw match score
        cv::putText(output_image, cv::format("%.4f", match.first),
                    cv::Point(x1, y1 + 12), cv::FONT_HERSHEY_DUPLEX, 0.30, box_color);
    }

    // Draw FPS if provided
    if (fps >= 0.0f)
    {
        cv::putText(output_image, cv::format("FPS: %.2f", fps),
                    cv::Point(0, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(255, 255, 255), 2);
    }

    return output_image;
}

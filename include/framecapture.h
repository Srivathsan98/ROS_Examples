// File: framecapture.h
#ifndef FRAMECAPTURE_H
#define FRAMECAPTURE_H

#include <opencv2/opencv.hpp>
#include <mutex>
#include <atomic>
#include <thread>
class FrameCapture {
public:
    FrameCapture();
    void captureLoop(std::atomic<bool>& running);
    cv::Mat getLatestFrame();
    // void visualize(const cv::Mat& frame, const std::vector<cv::Rect>& faces, const std::vector<std::string>& names);
    cv::Mat visualize(const cv::Mat& image,
                  const cv::Mat& faces,
                  const std::vector<std::pair<double, bool>>& matches,
                  const float fps = -0.1F,
                  const cv::Size& target_size = cv::Size(512, 512));
    std::pair<int, int> getFrameSize(const cv::Mat& frame);
    std::pair<int, int> getCameraSize();

private:
    cv::VideoCapture cap;
    cv::Mat latestFrame;
    std::mutex mtx;
    std::vector<std::tuple<double, bool, int>> matches;
};

#endif // FRAMECAPTURE_H
// File: facerecognizer.h
#ifndef FACERECOGNIZER_H
#define FACERECOGNIZER_H

#include <opencv2/opencv.hpp>
#include <opencv2/face.hpp>

class FaceRecognizer {
public:
    // FaceRecognizer(const std::string& modelPath);
    FaceRecognizer(const std::string &modelPath,
                               const int backentID,
                               const int targetID,
                               const int distance_type);
    // std::string recognize(const cv::Mat& frame, const cv::Rect& face);
    cv::Mat extractfeatures(const cv::Mat &original_image, const cv::Mat &face_image);
    std::pair<double, bool> matchFeatures(const cv::Mat &target_features, const cv::Mat &query_features);

    // cv::Mat extractfeatures(const cv::Mat &original_image, const cv::Mat &face_boxes);
    // std::vector<std::pair<double, bool>> matchFeatures(const cv::Mat &target_features, const cv::Mat &query_features_batch);

private:
    cv::Ptr<cv::FaceRecognizerSF> recognizer;
    cv::FaceRecognizerSF::DisType distancer_type;
    cv::Mat target_embedding;
    double threshold_cosine = 0.363;
    double threshold_norml2 = 1.128;
};

#endif // FACERECOGNIZER_H
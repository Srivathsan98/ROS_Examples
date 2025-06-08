#include "facerecognizer.h"

FaceRecognizer::FaceRecognizer(const std::string &modelPath,
                               const int backentID,
                               const int targetID,
                               const int distance_type)
    : distancer_type(static_cast<cv::FaceRecognizerSF::DisType>(distance_type))
{
    recognizer = cv::FaceRecognizerSF::create(modelPath, "", backentID, targetID);
}

cv::Mat FaceRecognizer::extractfeatures(const cv::Mat &original_image, const cv::Mat &face_image)
{
    cv::Mat target_aligned;
    recognizer->alignCrop(original_image, face_image, target_aligned);
    cv::Mat target_features;
    recognizer->feature(target_aligned, target_features);
    return target_features.clone();
}

std::pair<double, bool> FaceRecognizer::matchFeatures(const cv::Mat &target_features, const cv::Mat &query_features)
{
    const double score = recognizer->match(target_features, query_features, distancer_type);
    if (distancer_type == cv::FaceRecognizerSF::DisType::FR_COSINE)
    {
        return std::make_pair(score, score >= threshold_cosine);
    }
    return {score, score <= threshold_norml2};
}
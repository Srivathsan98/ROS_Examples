// File: facedetector.cpp
#include "facedetector.h"
/************************************************************************************************************************* */
// FaceDetector::FaceDetector(const std::string& modelPath) {
//     detector = cv::FaceDetectorYN::create(modelPath, "", cv::Size(320, 320));
//     detector->setInputSize(cv::Size(1280, 720));
// }

// std::vector<cv::Rect> FaceDetector::detectFaces(const cv::Mat& frame) {
//     std::vector<cv::Rect> results;
//     cv::Mat faces;
//     detector->detect(frame, faces);
//     for (int i = 0; i < faces.rows; ++i) {
//         cv::Rect r(faces.at<float>(i, 0), faces.at<float>(i, 1), faces.at<float>(i, 2), faces.at<float>(i, 3));
//         results.push_back(r);
//     }
//     return results;
// }
/********************************************************************************************************************** */
FaceDetector::FaceDetector(const std::string &modelPath, const cv::Size &inputSize,
                           const float confidenceThreshold, const float nmsThreshold,
                           const int topK, const int backentID, const int targetID)
{
    detector = cv::FaceDetectorYN::create(modelPath, "", inputSize,
                                          confidenceThreshold, nmsThreshold, topK, backentID, targetID);
}
cv::Mat FaceDetector::infer(const cv::Mat &image)
{
    cv::Mat output;
    detector->detect(image, output);
    return output;
}
void FaceDetector::setFrameInputSize(const cv::Size &size)
{
    detector->setInputSize(size);
}
void FaceDetector::setdetectionTopK(int topK)
{
    detector->setTopK(topK);
}
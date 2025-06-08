// File: facedetector.cpp
#include "facedetector.h"

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
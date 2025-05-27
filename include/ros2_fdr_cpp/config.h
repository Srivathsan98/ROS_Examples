#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include <string>

//configs
std::string facedetection_modelpath = "/home/pvsp/ros2_ws/src/ros2_fdr_cpp/models/face_detection_model.onnx";
std::string facerecognition_modelpath = "/home/pvsp/ros2_ws/src/ros2_fdr_cpp/models/face_recognition_model.onnx";
std::string target_image_path = "/home/pvsp/ros2_ws/src/ros2_fdr_cpp/models/target_image.jpg";
std::string target_image_name = "target.jpg";
#endif
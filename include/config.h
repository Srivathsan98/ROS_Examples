#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

// Declare variables as extern
extern std::string facedetection_modelpath;
extern std::string facerecognition_modelpath;
extern std::string target_image_path;
extern std::string target_image_name;
extern std::vector<std::string> target_image_paths;

//configs
#endif
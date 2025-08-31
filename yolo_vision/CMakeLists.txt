cmake_minimum_required(VERSION 3.8)
project(yolo_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(yolo_detect_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Onnxruntime
set(ONNXRUNTIME_DIR /usr/local/onnxruntime-linux-x64-gpu-1.22.0)
# find_library(ONNXRUNTIME_LIB onnxruntime REQUIRED)
include_directories(${ONNXRUNTIME_DIR}/include)
link_directories(${ONNXRUNTIME_DIR}/lib)

# Camera Publisher Node
add_executable(camera_pub src/camera_pub.cpp)
ament_target_dependencies(camera_pub rclcpp sensor_msgs cv_bridge OpenCV image_transport)

# Object Detection Node
add_executable(object_detection src/object_detection.cpp)
ament_target_dependencies(object_detection rclcpp sensor_msgs cv_bridge OpenCV yolo_detect_interfaces ament_index_cpp)
target_link_libraries(object_detection ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so)

# Subscriber Node
add_executable(obj_subscriber src/obj_subscriber.cpp)
ament_target_dependencies(obj_subscriber rclcpp yolo_detect_interfaces)

install(TARGETS
  camera_pub
  object_detection
  obj_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# Install models directory
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

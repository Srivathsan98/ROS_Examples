cmake_minimum_required(VERSION 3.8)
project(smart_detect)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Object.msg"
  "msg/Objects.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

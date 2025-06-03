# cmake_minimum_required(VERSION 3.8)
# project(ros2_fdr_cpp)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(image_transport REQUIRED)
# find_package(OpenCV REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()

# cmake_minimum_required(VERSION 3.8)
# project(ros2_fdr_cpp)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # Find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(OpenCV REQUIRED)

# # Include directories
# include_directories(
#   include
#   ${OpenCV_INCLUDE_DIRS}
# )

# # Add executable
# add_executable(face_recognition_node
#   src/main.cpp
#   src/FaceRecognitionNode.cpp
#   src/VideoRecordingNode.cpp
#   src/framecapture.cpp
#   src/facedetector.cpp
#   src/facerecognizer.cpp
# )

# # Link libraries
# target_link_libraries(face_recognition_node
#   ${OpenCV_LIBS}
# )

# # Install
# install(TARGETS
#   face_recognition_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# ament_package()


# // face_recognition_node/CMakeLists.txt

cmake_minimum_required(VERSION 3.8)
project(ros2_fdr_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Add executable
add_executable(face_recognition_node
  src/main.cpp
  src/FaceRecognitionNode.cpp
  src/VideoRecordingNode.cpp
  src/framecapture.cpp
  src/facedetector.cpp
  src/facerecognizer.cpp
)

# Link libraries
target_link_libraries(face_recognition_node
  ${OpenCV_LIBS}
)

# Add dependencies
ament_target_dependencies(face_recognition_node
  rclcpp
  sensor_msgs
  cv_bridge
)

# Install
install(TARGETS
  face_recognition_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

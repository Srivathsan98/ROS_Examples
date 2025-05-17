# cmake_minimum_required(VERSION 3.16)

# project(Qt_ROS_CAM_STRM_CPP VERSION 0.1 LANGUAGES CXX)

# set(CMAKE_AUTOUIC ON)
# set(CMAKE_AUTOMOC ON)
# set(CMAKE_AUTORCC ON)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
# find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

# set(PROJECT_SOURCES
#         main.cpp
#         mainwindow.cpp
#         mainwindow.h
#         mainwindow.ui
# )

# if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
#     qt_add_executable(Qt_ROS_CAM_STRM_CPP
#         MANUAL_FINALIZATION
#         ${PROJECT_SOURCES}
#     )
# # Define target properties for Android with Qt 6 as:
# #    set_property(TARGET Qt_ROS_CAM_STRM_CPP APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
# #                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# # For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
# else()
#     if(ANDROID)
#         add_library(Qt_ROS_CAM_STRM_CPP SHARED
#             ${PROJECT_SOURCES}
#         )
# # Define properties for Android with Qt 5 after find_package() calls as:
# #    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
#     else()
#         add_executable(Qt_ROS_CAM_STRM_CPP
#             ${PROJECT_SOURCES}
#         )
#     endif()
# endif()

# target_link_libraries(Qt_ROS_CAM_STRM_CPP PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)

# # Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# # If you are developing for iOS or macOS you should consider setting an
# # explicit, fixed bundle identifier manually though.
# if(${QT_VERSION} VERSION_LESS 6.1.0)
#   set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.Qt_ROS_CAM_STRM_CPP)
# endif()
# set_target_properties(Qt_ROS_CAM_STRM_CPP PROPERTIES
#     ${BUNDLE_ID_OPTION}
#     MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
#     MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
#     MACOSX_BUNDLE TRUE
#     WIN32_EXECUTABLE TRUE
# )

# include(GNUInstallDirs)
# install(TARGETS Qt_ROS_CAM_STRM_CPP
#     BUNDLE DESTINATION .
#     LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
# )

# if(QT_VERSION_MAJOR EQUAL 6)
#     qt_finalize_executable(Qt_ROS_CAM_STRM_CPP)
# endif()

# cmake_minimum_required(VERSION 3.16)
# project(Qt_ROS_CAM_STRM_CPP)

# # Qt Setup
# set(CMAKE_AUTOUIC ON)
# set(CMAKE_AUTOMOC ON)
# set(CMAKE_AUTORCC ON)
# set(CMAKE_INCLUDE_CURRENT_DIR ON)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # ROS 2 + OpenCV + Qt
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(OpenCV REQUIRED)

# find_package(QT NAMES Qt6 REQUIRED COMPONENTS Widgets)
# find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

# include_directories(
#   include
#   ${OpenCV_INCLUDE_DIRS}
# )

# set(PROJECT_SOURCES
#   src/main.cpp
#   src/mainwindow.cpp
#   include/qt_camera_gui/mainwindow.h
#   resources/mainwindow.ui
# )

# add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})

# target_link_libraries(${PROJECT_NAME}
#   Qt${QT_VERSION_MAJOR}::Widgets
#   ${OpenCV_LIBS}
# )

# ament_target_dependencies(${PROJECT_NAME}
#   rclcpp
#   sensor_msgs
#   cv_bridge
# )

# install(TARGETS ${PROJECT_NAME}
#   DESTINATION lib/${PROJECT_NAME}
# )

# ament_package()


cmake_minimum_required(VERSION 3.16)
project(Qt_ROS_CAM_STRM_CPP)

# Qt Setup
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ROS 2 + OpenCV
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Qt6 Setup
# If using Qt6, ensure to use correct naming
find_package(Qt6 REQUIRED COMPONENTS Widgets)

# Add include dirs
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_AUTOUIC_SEARCH_PATHS
  ${CMAKE_CURRENT_SOURCE_DIR}/resources
)

# Source files for Qt GUI
set(PROJECT_SOURCES
  src/main.cpp
  src/mainwindow.cpp
  include/qt_camera_gui/mainwindow.h
  resources/mainwindow.ui
)

# Qt GUI Executable
add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})

# Link libraries for Qt GUI
target_link_libraries(${PROJECT_NAME}
  Qt6::Widgets
  ${OpenCV_LIBS}
)

# ROS 2 dependencies for Qt GUI
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  cv_bridge
)

# Camera publisher executable
add_executable(camera_publisher src/camera_publisher.cpp)
target_link_libraries(camera_publisher ${OpenCV_LIBS})
ament_target_dependencies(camera_publisher
  rclcpp
  sensor_msgs
  cv_bridge
)

# Install targets
install(TARGETS ${PROJECT_NAME} camera_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

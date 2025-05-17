This code is used to communicate between Qt and ROS2 for capturing live frames from camera using ros2 and displaying it on a qt screen. there are 2 ways of doing it.
one is the publish and subscribe method where on one terminal we run the publisher that creates the ros topic for camera/image_raw
and another one terminal where we run the qt application that subscribes to the node and displays the incoming stream.

else
we can also add the publish code to the source and run.

to build and run
colcon build --package-select Qt_ROS_CAM_STRM_CPP
source the files(both the build dir and the ws dir)
run the exec using: 
one terminal  - ros2 run Qt_ROS_CAM_STRM_CPP camera_publisher
another terminal - ros2 run Qt_ROS_CAM_STRM_CPP Qt_ROS_CAM_STRM_CPP

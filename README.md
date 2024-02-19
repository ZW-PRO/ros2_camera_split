# ros2_camera_split

# Introduction
This is a feature package for dividing stereo cameras, which means dividing a complete image of the setreo camera into left and right cameras for ROS2 to use.
# Requirements
OpenCV == 3.2.0 (Tested in opencv==4.5, it causes bugs)
ROS2
# How to install
cd ros2_split_node  
colcon build
# How to use
ros2 run camera_split camera_split_node --ros-args -p left_cam_file:=file:///home..../left.yaml -p right_cam_firl:=file:///home/..../right.yaml

# If you have any question, welcome to leave a message

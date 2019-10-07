#!/usr/bin/env bash

# 7/10/2019
cd ~
mkdir -p catkin_ws/src
cd catkin_ws/src

git clone https://github.com/ros-drivers/usb_cam.git

cd ..

catkin_make

source /home/${USER}/catkin_ws/devel/setup.bash


# start ros core to publish camera
roscore &

# start usb_cam
rosrun usb_cam usb_cam_node _video_device:=/dev/video0

# to view image:
rosrun image_view image_view image:=/usb_cam/image_raw


# to calibrate camera with out chess board
rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.074 ima:=/usb_cam/image_raw camera:=/usb_cam

# opencv
git clone git@github.com:ros-drivers/video_stream_opencv.git



# start with orb_slam2
cd /home/${USER}/catkin_ws/src
git clone https://github.com/NescobarAlopLop/ORB_SLAM2.git

cd ORB_SLAM2
./build.sh


# monocular ros
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/${USER}/catkin_ws/src/ORB_SLAM2/Examples/ROS

./build_ros.sh


# strart ros server
roscore

# run video stream
roslaunch video_stream_opencv camera.launch

# run mono example
rosrun ORB_SLAM2 Mono /home/${USER}/catkin_ws/src/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/ge/catkin_ws/src/ORB_SLAM2/Examples/Monocular/EuRoC.yaml
rosrun ORB_SLAM2 Mono /home/${USER}/catkin_ws/src/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/ge/catkin_ws/src/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Asus.yaml


rosrun ORB_SLAM2 Mono /home/${USER}/catkin_ws/src/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/ge/.ros/camera_info/head_camera.yaml


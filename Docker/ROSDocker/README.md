
- connect usb cam, it should be recognized under /dev/video0
if not please update docker-compose accordingly, under talker container

- run `start_ros_orb_docker_with_gui.sh`

TODO:
for next time, start talker container with roslaunch video_stream_opencv camera.launch
if think i should first install the missing package

then run listener with
rosrun ORB_SLAM2 Mono <path_to_voc_folder>/ORBVoc.txt <path_to_cam_settings>/head_camera.yaml
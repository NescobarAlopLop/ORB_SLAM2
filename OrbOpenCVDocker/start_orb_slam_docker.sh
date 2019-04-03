# docker run -v /home/george/orb_slam_inpus/:/mnt/mydata -it orbslam:v1 /bin/bash
docker run -v /home/george/orb_slam_inpus/:/mnt/mydata -it  --rm \
			-e DISPLAY=$DISPLAY \
			-v /tmp/.X11-unix:/tmp/.X11-unix \
			orbslam:v1 /bin/bash
#			orbslam:v1 /root/ORB_SLAM2/Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt /root/ORB_SLAM2/Examples/Monocular/TUMX.yaml /mnt/mydata/rgbd_dataset_freiburg1_xyz


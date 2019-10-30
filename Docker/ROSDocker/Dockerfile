# This is an auto generated Dockerfile for ros:ros-core
# generated from docker_images/create_ros_core_image.Dockerfile.em
FROM ubuntu:bionic

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO melodic
RUN apt-get update && apt-get install -y \
    ros-melodic-ros-tutorials \
    ros-melodic-common-tutorials \
    ros-melodic-ros-core=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

# installing more deps
RUN apt-get update && apt-get install -y libeigen3-dev libblas-dev liblapack-dev libglew-dev

# installing opencv
RUN apt-get -y install libopencv-dev python3-opencv

# installing Pangolin
RUN cd ~ && \
    git clone https://github.com/stevenlovegrove/Pangolin && \
    cd Pangolin && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j4 && \
    make install


# copy ORB-SLAM2 from current directory
RUN printenv | grep ROS
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash; \
    mkdir -p ~/catkin_ws/src; \
    cd ~/catkin_ws/; \
    catkin_make; \
    source devel/setup.bash'

RUN cd ~/catkin_ws/src/ && \
    git clone https://github.com/NescobarAlopLop/ORB_SLAM2.git && \
    cd ORB_SLAM2 && \
    chmod +x build.sh && \
    ./build.sh

RUN apt-get install -y ros-melodic-rqt-tf-tree

#ENV ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/catkin_ws/src/ORB_SLAM2/Examples/ROS
#RUN export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/catkin_ws/src/ORB_SLAM2/Examples/ROS && \
#RUN echo $ROS_PACKAGE_PATH
ENV ROS_PACKAGE_PATH=/opt/ros/melodic/share:~/catkin_ws/src/ORB_SLAM2/Examples/ROS:~/catkin_ws/src
#RUN echo $ROS_PACKAGE_PATH
#RUN roscore
#RUN rosrun tf tf_monitor
#RUN apt install -y ros-melodic-desktop-full
RUN apt install -y ros-melodic-desktop

#RUN /bin/bash -c 'export ROS_PACKAGE_PATH=/opt/ros/melodic/share:~/catkin_ws/src/ORB_SLAM2/Examples/ROS:~/catkin_ws/src && \
RUN /bin/bash -c 'echo  $ROS_PACKAGE_PATH && \
    cd ~/catkin_ws/ && \
    source devel/setup.bash && \
    cd src/ORB_SLAM2 && \
    chmod +x build_ros.sh && \
    ./build_ros.sh'
RUN echo  $ROS_PACKAGE_PATH

#source /opt/ros/melodic/setup.bash && \

#    rosdep update; \

## Install desktop environment
#RUN apt-get install -y xserver-xorg-core \
#    xinit lxsession desktop-file-utils \
##     rpd-icons \
##    gtk2-engines-clearlookspix \
#    matchbox-keyboard
#
## disable lxpolkit popup warning
#RUN mv /usr/bin/lxpolkit /usr/bin/lxpolkit.bak

#RUN echo "#!/bin/bash" > /etc/X11/xinit/xserverrc \
#  && echo "" >> /etc/X11/xinit/xserverrc \
#  && echo 'exec /usr/bin/X -s 0 dpms -nolisten tcp "$@"' >> /etc/X11/xinit/xserverrc

# setup entrypoint
COPY ./test_webcam.py /
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

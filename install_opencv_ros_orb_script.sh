#!/usr/bin/env bash


echo "[ ** ] updating"
sudo apt-get update
sudo apt-get -y upgrade


echo "[ ** ] installing cmake and git"
sudo apt-get -y install cmake git


echo "[ ** ] installing opencv"
sudo apt-get -y install libopencv-dev python3-opencv

echo "[ ** ] installing more deps"
sudo apt-get -y install libeigen3-dev libblas-dev liblapack-dev libglew-dev


echo "[ ** ] installing Pangolin"
cd ~
git clone https://github.com/stevenlovegrove/Pangolin
cd Pangolin
mkdir build
cd build
cmake ..
make -j4
make install
echo "finished making Pangolin"


echo "[ ** ] clone ORB-SLAM2 from github"
cd ~
git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
cd ORB_SLAM2

chmod +x build.sh
./build.sh


echo "[ ** ] Installing ROS"
# according to this tutorial https://www.hackster.io/dmitrywat/ros-melodic-on-raspberry-pi-4-debian-buster-rplidar-a1m8-0d63d1
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

sudo apt-get -y update
sudo apt-get -y upgrade

sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake

sudo rosdep init
rosdep update

mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws

rosinstall_generator desktop --rosdistro melodic --deps --wet-only --tar > melodic-desktop-wet.rosinstall
wstool init -j4 src melodic-desktop-wet.rosinstall

# if error continuew with this command
# wstool update -j 4 -t src


# fixing issues according to tutorial
# https://www.seeedstudio.com/blog/2019/08/01/installing-ros-melodic-on-raspberry-pi-4-and-rplidar-a1m8/

mkdir -p ~/ros_catkin_ws/external_src
cd ~/ros_catkin_ws/external_src
wget http://sourceforge.net/projects/assimp/files/assimp-3.1/assimp-3.1.1_no_test_models.zip/download -O assimp-3.1.1_no_test_models.zip
unzip assimp-3.1.1_no_test_models.zip
cd assimp-3.1.1
cmake .
make
sudo make install

sudo apt-get -y install libogre-1.9-dev
exit 1
echo "stopped install HERE" find this line in script and continue manually"


# stop here!
# run
cd ~/ros_catkin_ws/
find -type f -print0 | xargs -0 grep 'boost::posix_time::milliseconds' | cut -d: -f1 | sort -u  

# look for 'boost::posix_time::milliseconds'
# fix resulting files according to this convention
# replace calls like this:
# boost::posix_time::milliseconds(loop_duration.toSec() * 1000.0f));

# with:
# boost::posix_time::milliseconds(int(loop_duration.toSec() * 1000.0f)));

# AND
# calls like this:
# boost::posix_time::milliseconds(1000.0f)

# with:
# boost::posix_time::milliseconds(1000)

# continue here

# long time
rosdep install --from-paths src --ignore-src --rosdistro melodic -y
# long time
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic -j2


echo "[ ** ] Finished ROS install"
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get install python-empy

##########################################

# dependecies for video transfer (ogg)
cd ~

wget http://downloads.xiph.org/releases/ogg/libogg-1.3.1.tar.gz
tar -xf libogg-1.3.1.tar.gz 
cd libogg-1.3.1/
./configure --host=arm-unknown-linux-gnueabi --enable-static
make
sudo make install

cd ~

wget http://downloads.xiph.org/releases/vorbis/libvorbis-1.3.3.tar.gz
tar -xf libvorbis-1.3.3.tar.gz
cd libvorbis-1.3.3/
./configure --host=arm-unknown-linux-gnueabi --enable-static
make
sudo make install

cd ~

wget http://downloads.xiph.org/releases/theora/libtheora-1.1.1.tar.bz2
tar -xf libtheora-1.1.1.tar.bz2 
cd libtheora-1.1.1/
./configure --host=arm-unknown-linux-gnueabi --enable-static
make
sudo make install

exit(1)
echo "clean examples from makefile, didn\'t compile for me"



###############################################

cd ~
mkdir -p catkin_ws/src
cd catkin_ws
catkin_make
cd src
git clone https://github.com/UbiquityRobotics/raspicam_node.git
git clone https://github.com/ros-perception/image_transport_plugins.git
git clone https://github.com/ros-perception/image_common.git

# to see and test camera
roslaunch raspicam_node camerav2_1280x960.launch &
rosrun raspicam_node imv_view.py
# additional info at git repo raspicam_node

####################

cd ~/catkin_ws
catkin_make

######################
# hopefully run with ros:
echo "export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/ORB_SLAM2/Examples/ROS" >> ~/.bashrc
source ~/.bashrc
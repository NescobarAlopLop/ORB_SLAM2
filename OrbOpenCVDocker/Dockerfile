#FROM        ubuntu:16.04
FROM opless/ubuntu-16.04-opencv-3.2.0
MAINTAINER  George Kesaev
ENV TZ=Israel/Haifa
# ENV TZ=America/Los_Angeles
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

#install opencv
#RUN apt-get -y update && apt-get -y upgrade
RUN apt-get -y update
RUN apt-get -y install build-essential wget nano 
# RUN apt-get -y install build-essential perl wget nano
RUN apt-get -y install cmake git libgtk2.0-dev pkg-config libavutil-dev libpostproc-dev libavdevice-dev libsdl-dev
RUN apt-get -y install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
RUN apt-get -y install libeigen3-dev
RUN apt-get -y install libgl1-mesa-dev
RUN apt-get -y install libblas-dev liblapack-dev
RUN apt-get -y install libglew-dev

# The following command is needed to process images:
RUN apt-get -y install python-dev libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev
#RUN apt-get install libjasper-dev

# To process videos
RUN apt-get -y install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
RUN apt-get -y install libxvidcore-dev libx264-dev

# for gui
RUN apt-get -y install libgtk-3-dev

# for optimization
RUN apt-get -y install libatlas-base-dev gfortran pylint

# build for both python 2 and 3
RUN apt-get -y install software-properties-common
RUN add-apt-repository -y ppa:fkrull/deadsnakes
RUN apt-get -y update
RUN apt-get -y install python2.7-dev python3.6-dev
# RUN apt-get -y install python2.7 python3.6
RUN apt-get -y install python-numpy python-scipy python3-numpy python3-scipy

RUN apt-get -y install qt5-default

#install opencv
#RUN cd && wget https://github.com/opencv/opencv/archive/3.2.0.zip -O opencv-3.2.0.zip
#RUN cd && wget https://github.com/opencv/opencv_contrib/archive/3.2.0.zip -O opencv_contrib-3.2.0.zip
#RUN apt-get -y install unzip
#RUN cd && unzip opencv-3.2.0.zip
#RUN cd && unzip opencv_contrib-3.2.0.zip
#RUN cd && mkdir -p opencv-3.2.0/build

#RUN cd && cd opencv-3.2.0/build && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-3.4.0/modules -DOPENCV_ENABLE_NONFREE=True -D WITH_CUDA=OFF ..

#RUN cd && cd opencv-3.2.0/build && make -j
#RUN cd && cd opencv-3.2.0/build && make install
#RUN echo "include /usr/local/lib" >> /etc/ld.so.conf
#RUN ldconfig

#install glew
RUN apt-get -y install build-essential libxmu-dev libxi-dev libgl-dev cmake git

RUN apt-get -y install zip && apt-get -y install unzip

RUN cd && wget https://raw.githubusercontent.com/NescobarAlopLop/miscellaneous/master/glew-2.1.0.zip && unzip glew-2.1.0.zip
#RUN cd && cd glew && make -j && make install && make clean
#RUN cd && cd glew-2.1.0/build && cmake ./cmake && make -j4
RUN cd && cd glew-2.1.0/build && cmake ./cmake && make -j4 && make install && make clean

#RUN cd && cd glew && make
#RUN cd && make install && make clean

#install pangolin
RUN cd && git clone https://github.com/stevenlovegrove/Pangolin.git
RUN cd && cd Pangolin && mkdir build && cd build && cmake .. && make -j

#Get orb slam2
RUN cd && git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
RUN cd && cd ORB_SLAM2 && chmod +x ./build.sh && ./build.sh


# ENV HOME /home/developer
ENV HOME /root/ORB_SLAM2

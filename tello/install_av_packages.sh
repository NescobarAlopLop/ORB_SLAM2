#!/usr/bin/env bash
# General dependencies
sudo apt-get install -y python3-dev python3.6-dev python3.7-dev pkg-config

# Library components
sudo apt-get install -y \
    libavformat-dev libavcodec-dev libavdevice-dev \
    libavutil-dev libswscale-dev libswresample-dev libavfilter-dev \
    libavresample-dev

# for pi only: maybe not sure
sudo apt-get install -y libatals-base-dev
sudo apt-get install -y libjasper-dev
sudo apt-get install -y libqtgui4
sudo apt-get install -y python3-pyqt5
sudo apt-get install -y python3-pyqt4

# for all
sudo apt install ffmpeg

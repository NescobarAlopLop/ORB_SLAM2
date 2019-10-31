#!/usr/bin/env bash
# General dependencies
sudo apt-get install -y python-dev pkg-config

# Library components
sudo apt-get install -y \
    libavformat-dev libavcodec-dev libavdevice-dev \
    libavutil-dev libswscale-dev libswresample-dev libavfilter-dev

# for pi only:
sudo apt-get install libatals-base-dev
sudo apt-get install libjasper-dev
sudo apt-get install libqtgui4
sudo apt-get install python3-pyqt5
sudo apt-get install python3-pyqt4
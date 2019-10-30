#!/usr/bin/env bash
# note: not very safe, please consult http://wiki.ros.org/docker/Tutorials/GUI
xhost +local:root
docker-compose up -d

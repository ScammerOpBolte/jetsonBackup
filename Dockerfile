FROM nvidia/cuda:11.3.0-base-ubuntu20.04
 
# Minimal setup

RUN apt-get update \
 && apt-get install -y locales lsb-release \
 && dpkg-reconfigure locales \
 && apt-get install -y python3-catkin-tools python3-osrf-pycommon
 
# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
 && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
 && apt-get update \
 && apt-get install -y --no-install-recommends ros-noetic-desktop-full \
 && apt-get install -y --no-install-recommends python3-rosdep \
 && rosdep init \
 && rosdep fix-permissions \
 && rosdep update \
 && echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \ 
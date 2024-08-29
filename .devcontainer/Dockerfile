# @license BSD-3 https://opensource.org/licenses/BSD-3-Clause
# Copyright (c) 2021, Institute of Automatic Control - RWTH Aachen University
# All rights reserved.

# Choose from melodic or noetic tags
FROM ros:noetic-robot

# Add your own additional packages
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
  && apt-get install -y \
  # Check if GUI and 3D acceleration works
  mesa-utils x11-apps \
  # Nice plots for debugging
  ros-${ROS_DISTRO}-plotjuggler-ros \
  && apt-get upgrade -y \
  && rm -rf /var/lib/apt/lists/*

# git-lfs
RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash && \
  apt-get update && \
  apt-get install git-lfs -y --no-install-recommends && \
  rm -rf /var/lib/apt/lists/* && \
  git lfs install
# ROS additional essential packages
RUN apt-get update && apt-get install -y --no-install-recommends\
  python3-catkin-tools python3-catkin-lint python3-rosdep python3-rosinstall python3-rosinstall-generator python3-vcstool python3-wstool build-essential python3-osrf-pycommon \
  ros-${ROS_DISTRO}-ros-control ros-${ROS_DISTRO}-ros-controllers ros-${ROS_DISTRO}-rqt-controller-manager && \
  rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=dialog

# Setup non-root user to solve file permission problems with rootfull docker
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN groupadd --gid ${USER_GID} $USERNAME \
  && useradd --uid ${USER_UID} --gid ${USER_GID} -m $USERNAME \
  && apt-get update \
  && apt-get install -y --no-install-recommends sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME

# Autocompletion for root user
RUN echo "if [ -f /etc/bash_completion ] && ! shopt -oq posix; then\n   . /etc/bash_completion\nfi" >> /root/.bashrc

# Create for vscode 
RUN sudo mkdir -p /workspace/src && chown -R vscode:vscode /workspace
# Use non-root user to avoid permission problems
USER vscode

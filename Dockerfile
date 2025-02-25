# syntax=docker/dockerfile:1.3

# Use official ROS 2 Humble image (Ubuntu 22.04)
FROM ros:humble AS bmw_robotics_task

# Switch default shell to Bash for 'source' commands
SHELL ["/bin/bash", "-c"]

# Environment variables
ENV LANG=C.UTF-8
ENV ROS_DISTRO=humble
WORKDIR /bmw_ws

RUN apt-get update && \
  apt-get install -y \
  python3-pip python3-setuptools python3-numpy python3-dev python3-pybind11 cython3 \
  python3-colcon-common-extensions \
  ros-humble-rclcpp \
  ros-humble-std-msgs \
  ros-humble-sensor-msgs \
  ros-humble-generate-parameter-library \
  ros-humble-cv-bridge \
  libopencv-dev && \
  pip install --no-cache-dir --upgrade "setuptools==65.5.0" numpy && \
  rm -rf /usr/src/gtest /usr/src/gmock || true && \
  rm -rf /var/lib/apt/lists/*

# Copy your workspace source code into the container
COPY . /bmw_ws

# Build the ROS 2 workspace
RUN source /opt/ros/humble/setup.bash && \
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Automatically source the ROS environment in new shells
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
  echo "source /bmw_ws/install/setup.bash" >> /root/.bashrc

# (Optional) Default command
CMD ["bash"]

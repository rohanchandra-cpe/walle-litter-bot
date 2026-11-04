# Base image: ROS 2 Humble on Ubuntu 22.04
# Use linux/arm64 for Apple Silicon Macs, linux/amd64 for Intel Macs and Windows
FROM --platform=linux/arm64 arm64v8/ros:humble

# Install Python pip and ROS 2 Python client library
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-rclpy \
    && rm -rf /var/lib/apt/lists/*

# Pre-install Python dependencies (used in the ROSA guide)
RUN pip3 install --no-cache-dir \
    ollama \
    jpl-rosa \
    'langchain-ollama<1.0' \
    'langchain-core>=0.3.52,<1.0' \
    'mcp>=1.9.2,<2.0' \
    'langchain-mcp-adapters==0.1.10' \
    'chromadb==0.6.3' \
    'posthog>=2.4,<4.0' \
    requests

# WALL-E pilot (walle-pilot/): the EKF, ROS bag recording, and Depth Anything V2.
# CPU-only PyTorch avoids multi-gigabyte CUDA libraries.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-robot-localization \
    ros-humble-tf2-ros \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-default-plugins \
    python3-yaml \
    git \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir torch --index-url https://download.pytorch.org/whl/cpu
RUN pip3 install --no-cache-dir \
    'transformers>=4.45,<5' \
    'scipy>=1.13' \
    'matplotlib>=3.8' \
    pillow
# The mounted folder belongs to the host user; let git read its commit for the logs
RUN git config --system --add safe.directory '*'

# Source ROS 2 automatically in every shell
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
ENV PYTHONPATH="/opt/ros/humble/lib/python3.10/dist-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:${PYTHONPATH}"
ENV LD_LIBRARY_PATH="/opt/ros/humble/lib:${LD_LIBRARY_PATH}"
ENV AMENT_PREFIX_PATH="/opt/ros/humble"

WORKDIR /workspace
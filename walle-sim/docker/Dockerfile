# WALL-E in Gazebo: ROS 2 Humble + Gazebo Fortress + a browser desktop (noVNC),
# plus everything the pilot stack needs (robot_localization, rosbag2, Depth Anything V2,
# ROSA), so the robot's own scripts run against the simulator unchanged.
#
# Combines Module 6's Gazebo image with the repository's pilot Dockerfile. The ros:humble
# image is multi-architecture: Apple Silicon, Intel Macs and Windows PCs all work.
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# Gazebo Fortress, the ROS 2 <-> Gazebo bridge, robot model tools (as in Module 6)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gz \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-rqt-image-view \
    ros-humble-teleop-twist-keyboard \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Browser desktop: virtual screen, VNC server, noVNC, a small window manager
RUN apt-get update && apt-get install -y --no-install-recommends \
    xvfb \
    x11vnc \
    novnc \
    python3-websockify \
    openbox \
    xterm \
    mesa-utils \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Window rules: Gazebo opens maximized; the camera viewer opens large and on top
RUN mkdir -p /root/.config/openbox \
    && sed -e 's#<applications>#&<application name="ign-gazebo-gui"><maximized>yes</maximized></application>#' \
           -e 's#<applications>#&<application name="rqt_image_view"><position force="yes"><x>40</x><y>60</y></position><size><width>680</width><height>540</height></size><layer>above</layer></application>#' \
       /etc/xdg/openbox/rc.xml > /root/.config/openbox/rc.xml

# The pilot stack (same packages as the repository's Dockerfile): the EKF, bags, git
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-robot-localization \
    ros-humble-tf2-ros \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-default-plugins \
    python3-yaml \
    git \
    && rm -rf /var/lib/apt/lists/*

# ROSA and the Ollama connector (same versions as the pilot image)
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

# Depth Anything V2 on the CPU (no multi-gigabyte CUDA libraries)
RUN pip3 install --no-cache-dir torch --index-url https://download.pytorch.org/whl/cpu
RUN pip3 install --no-cache-dir \
    'transformers>=4.45,<5' \
    'scipy>=1.13' \
    'matplotlib>=3.8' \
    pillow

# WALL-E Phase 3 detection: YOLO-World (ultralytics) on the CPU, with CLIP for its text
# prompts. Pins checked together 2026-11-19 (numpy 2.2.6, torch 2.14.1+cpu). pip is upgraded
# so it can build CLIP from git; setuptools stays < 80 for colcon. torchvision comes from the
# CPU index so it matches torch (the PyPI build fails with "torchvision::nms does not exist").
# Weights live in walle-app/models/ (git-ignored); CLIP's ~/.cache/clip points there too, so
# a rebuild doesn't download them again. sync=False: no ultralytics usage analytics.
RUN apt-get update && apt-get install -y --no-install-recommends libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir -U 'pip==26.3' 'setuptools>=70,<80' wheel
RUN pip3 install --no-cache-dir 'torchvision==0.29.2' --index-url https://download.pytorch.org/whl/cpu
RUN pip3 install --no-cache-dir \
    'ultralytics==8.4.215' \
    'opencv-python==5.1.0.95' \
    'clip @ git+https://github.com/ultralytics/CLIP.git@7ffa84b3bfa40c42ecc1c77147a855e69cb2dd40'
RUN yolo settings sync=False >/dev/null \
    && mkdir -p /root/.cache && ln -s /workspace/walle-app/models/clip /root/.cache/clip

# The mounted folder belongs to the host user; let git read its commit for the logs
RUN git config --system --add safe.directory '*'

COPY start-desktop.sh /usr/local/bin/start-desktop.sh
RUN sed -i 's/\r$//' /usr/local/bin/start-desktop.sh \
    && chmod +x /usr/local/bin/start-desktop.sh

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

WORKDIR /workspace/walle-pilot
CMD ["/usr/local/bin/start-desktop.sh"]

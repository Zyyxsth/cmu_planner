FROM ros:humble-ros-base

# Install dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-joy \
    ros-humble-tf2-ros \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-compressed-image-transport \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-rviz2 \
    python3-colcon-common-extensions \
    python3-pip \
    libgoogle-glog-dev \
    libgflags-dev \
    libpcl-dev \
    libeigen3-dev \
    libboost-all-dev \
    tmux \
    net-tools \
    iputils-ping \
    socat \
    xauth \
    libglu1-mesa \
    libxrandr2 \
    libxi6 \
    libxcursor1 \
    libxinerama1 \
    libxxf86vm1 \
    libnss3 \
    libasound2 \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws

# Copy source code
COPY . /ros2_ws/src/

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /ros2_ws && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Setup environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Set library path for or-tools
ENV LD_LIBRARY_PATH=/ros2_ws/src/mtare_planner/tare_planner/or-tools/lib:$LD_LIBRARY_PATH

WORKDIR /ros2_ws

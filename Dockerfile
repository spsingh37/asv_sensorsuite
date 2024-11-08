FROM ros:humble-ros-core-jammy

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    wget \
    python3-pip vim \
    iputils-ping \
    tmux \
    libyaml-cpp-dev \
    libpcap-dev \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install setuptools==58.2.0

RUN git config --global user.email "suryasin@umich.edu"
RUN git config --global user.name "spsingh37" 

ARG ROS_DISTRO=humble

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

RUN apt-get update && apt-get upgrade -y && apt-get install -y \
        ros-$ROS_DISTRO-teleop-twist-keyboard \
        ros-$ROS_DISTRO-rviz2 \
        ros-$ROS_DISTRO-rviz-common \
        ros-$ROS_DISTRO-rviz-default-plugins \
        ros-$ROS_DISTRO-rviz-visual-tools \
        ros-$ROS_DISTRO-rviz-rendering \
        ros-$ROS_DISTRO-nav2-rviz-plugins \
        # for ffmpeg image transport
        ros-$ROS_DISTRO-cv-bridge \
        # allows compressed and theora encoded streams to be received over image_transport
        ros-$ROS_DISTRO-image-transport-plugins \
        # DepthAI
        ros-$ROS_DISTRO-depthai-descriptions && \
    apt-get upgrade -y && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Create the workspace
RUN mkdir -p lidar_ws/src
COPY rslidar_sdk /lidar_ws/src/rslidar_sdk
COPY rslidar_msg /lidar_ws/src/rslidar_msg

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    cd lidar_ws/src/rslidar_sdk && \
    git submodule init && \
    git submodule update && \
    cd ../.. && \
    colcon build

RUN apt-get update && \
    apt-get install -y software-properties-common && \
    rm -rf /var/lib/apt/lists/*
    
# install realsense packages
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
    add-apt-repository -y "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u && \
    apt-get install -y librealsense2-utils && apt-get install librealsense2-dev && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt install -y curl && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt install -y ros-humble-realsense2-*
    
# Copy the entrypoint script into the container
COPY entrypoint.sh entrypoint.sh

# Make the entrypoint script executable
RUN chmod +x entrypoint.sh

# Set the entrypoint
# ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
RUN ./entrypoint.sh

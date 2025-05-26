FROM osrf/ros:jazzy-desktop-full

SHELL [ "/bin/bash" , "-c" ]

# Upgrade all packages
RUN sudo apt update && sudo apt upgrade -y

# Install essential packages
#RUN sudo apt install -y wget
RUN sudo apt-get update; \
    apt-get upgrade -y; \
    apt-get install -y terminator x11-apps wget \
    libxcb-xinerama0 \
    libxcb-xinerama0-dev \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-render-util0 \
    libxkbcommon-x11-0 \
    libglu1-mesa \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Create overlay workspace
WORKDIR /root/ros_ws/src
COPY packages.repos .

RUN vcs import < packages.repos; \
    cd ..; \
    rosdep install --from-paths src --ignore-src -r -y; \
    source /opt/ros/${ROS_DISTRO}/setup.bash; \
    colcon build;

# Add sourcing ROS setup.bash to .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "export LIBGL_ALWAYS_SOFTWARE=1" >> /root/.bashrc
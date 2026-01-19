ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y -q --no-install-recommends \
    wget \
    curl \
    vim \
    build-essential \
    ca-certificates \
    gnupg2 \
    lsb-release \
    python3-pip \
    python3-tk \
    git \
    git-lfs \
    cmake \
    make \
    apt-utils \
    libpcl-dev \
    libfmt-dev \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rosbag2-cpp \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros_ws

COPY . /ros_ws/src/PointCloudCrafter

RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'

# Set entrypoint by sourcing overlay workspace
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> /root/.bashrc && \
    echo '. /ros_ws/install/setup.bash' >> /root/.bashrc && \
    echo '#!/bin/bash\nset -e\n\n# setup ros environment\nsource "/opt/ros/$ROS_DISTRO/setup.bash"\n. /ros_ws/install/setup.bash\nexec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD [ "bash" ]
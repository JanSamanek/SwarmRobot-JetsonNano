FROM ros:humble

SHELL [ "/bin/bash", "-c" ]

RUN apt-get update && apt-get install -y \
python3-colcon-common-extensions \
python3-rosdep \
build-essential \
nlohmann-json3-dev \
libeigen3-dev \
git \
&& rm -rf /var/lib/apt/lists/* 

RUN mkdir -p /ros2ws/src
WORKDIR /ros2_ws

RUN cd src/ && \
    git clone https://github.com/Slamtec/rplidar_ros.git -b ros2 && \
    git clone https://github.com/ajtudela/laser_segmentation.git

COPY ros2_ws/src ./src


RUN source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r --rosdistro humble -y && \
    colcon build --symlink-install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

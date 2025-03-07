FROM ros:humble

SHELL [ "/bin/bash", "-c" ]

RUN apt-get update && apt-get install -y \
python3-colcon-common-extensions \
python3-rosdep \
build-essential \
nlohmann-json3-dev \
libeigen3-dev \
libpcl-dev \
git \
&& rm -rf /var/lib/apt/lists/* 

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /ros2_ws/src

WORKDIR /ros2_ws

RUN cd src/ && \
    git clone https://github.com/Slamtec/rplidar_ros.git -b ros2 && \
    git clone https://github.com/ajtudela/laser_segmentation.git && \
    git clone https://github.com/dawan0111/Simple-2D-LiDAR-Odometry.git

COPY ros2_ws/src ./src
COPY entrypoint.sh ./entrypoint.sh
RUN chmod +x ./entrypoint.sh

USER $USERNAME
RUN sudo apt-get update && \
    source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r --rosdistro humble -y --skip-keys PCL --skip-keys Eigen3

USER root
RUN apt-get update && \
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

ENTRYPOINT ["./entrypoint.sh"]

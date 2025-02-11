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

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

RUN mkdir -p /ros2_ws/src

USER $USERNAME
WORKDIR /ros2_ws

RUN cd src/ && \
    git clone https://github.com/Slamtec/rplidar_ros.git -b ros2 && \
    git clone https://github.com/ajtudela/laser_segmentation.git

COPY ros2_ws/src ./src

RUN apt-get update && \
    source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r --rosdistro humble -y && \
    colcon build --symlink-install

COPY entrypoint.sh ./entrypoint.sh
RUN chmod +x ./entrypoint.sh

ENTRYPOINT ["./entrypoint.sh"]

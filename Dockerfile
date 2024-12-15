FROM ros:humble

SHELL [ "/bin/bash", "-c" ]

RUN apt-get update && apt-get install -y \
python3-colcon-common-extensions \
python3-rosdep \
build-essential \
&& rm -rf /var/lib/apt/lists/*

RUN mkdir -p /ros2ws/src
WORKDIR /ros2_ws

RUN rosdep update

COPY ros2_ws/src ./src

RUN source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

ENTRYPOINT ["/entrypoint.sh"]

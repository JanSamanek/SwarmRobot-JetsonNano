FROM ros:humble

SHELL [ "/bin/bash", "-c" ]

WORKDIR /ros2_ws

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    ros-humble-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update

COPY ros2_ws/src ./src

RUN source /opt/ros/humble/setup.bash && rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install
RUN source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash

ENTRYPOINT ["ros2 ", "launch", "controller", "controller.launch.py"]

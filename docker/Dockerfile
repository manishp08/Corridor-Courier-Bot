# ROS 2 Jazzy + Gazebo Harmonic + Nav2 + this workspace.
#   make docker && make docker-run
#   ros2 launch cc_navigation bringup.launch.py world:=glass_wall ablation:=D rviz:=true
FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz \
      ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-mppi-controller \
      ros-jazzy-robot-localization ros-jazzy-slam-toolbox \
      ros-jazzy-vision-msgs ros-jazzy-twist-mux ros-jazzy-teleop-twist-keyboard \
      ros-jazzy-xacro ros-jazzy-rosbag2-storage-mcap \
      python3-pip python3-scipy python3-matplotlib python3-opencv python3-pytest python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --break-system-packages onnxruntime evo

WORKDIR /ws
COPY src ./src
COPY sim ./sim
COPY Makefile ./
RUN . /opt/ros/jazzy/setup.sh && rosdep update && \
    rosdep install --from-paths src --ignore-src -y --skip-keys "python3-pytest" || true
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash" >> /root/.bashrc
CMD ["bash"]

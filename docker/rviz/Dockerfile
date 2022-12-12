FROM ros:noetic

RUN apt update && apt -y install xterm
RUN apt update && apt install -y ros-${ROS_DISTRO}-rviz

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
COPY rviz-config.rviz /app/rviz-config.rviz

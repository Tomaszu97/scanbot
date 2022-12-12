FROM ros:noetic
SHELL ["/bin/bash", "-c"]

RUN apt update && apt -y install xterm
RUN apt update && apt -y install python3-pip
RUN apt update && apt -y install ros-${ROS_DISTRO}-tf

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

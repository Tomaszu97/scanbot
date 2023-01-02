FROM ros:noetic
SHELL ["/bin/bash", "-c"]

RUN apt update && apt -y install xterm
RUN apt update && apt -y install python3-pip
RUN apt update && apt -y install ros-${ROS_DISTRO}-tf
RUN apt update && apt -y install ros-${ROS_DISTRO}-gazebo-ros-pkgs
RUN apt update && apt -y install vim
RUN apt update && apt -y install git

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

FROM ros:noetic

RUN apt update && apt -y install xterm
RUN apt update && apt -y install ros-${ROS_DISTRO}-tf
RUN apt update && apt -y install ros-${ROS_DISTRO}-xacro
RUN apt update && apt -y install ros-${ROS_DISTRO}-joint-state-publisher
RUN apt update && apt -y install ros-${ROS_DISTRO}-robot-state-publisher
RUN apt update && apt -y install ros-${ROS_DISTRO}-rviz
RUN apt update && apt -y install ros-${ROS_DISTRO}-slam-gmapping
RUN apt update && apt -y install ros-${ROS_DISTRO}-gazebo-ros-control
RUN apt update && apt -y install ros-${ROS_DISTRO}-effort-controllers
RUN apt update && apt -y install ros-${ROS_DISTRO}-joint-state-controller
RUN apt update && apt -y install ros-${ROS_DISTRO}-rqt-gui
RUN apt update && apt -y install ros-${ROS_DISTRO}-diff-drive-controller

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
WORKDIR /root

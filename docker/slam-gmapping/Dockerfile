FROM ros:noetic

RUN apt update && apt -y install xterm
RUN apt update && apt -y install ros-${ROS_DISTRO}-slam-gmapping

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

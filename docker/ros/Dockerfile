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
RUN apt update && apt -y install ros-${ROS_DISTRO}-velocity-controllers
RUN apt update && apt -y install ros-${ROS_DISTRO}-gazebo-ros-pkgs
RUN apt update && apt -y install ros-${ROS_DISTRO}-joy
RUN apt update && apt -y install ros-${ROS_DISTRO}-joy-teleop
RUN apt update && apt -y install ros-${ROS_DISTRO}-teleop-twist-joy
RUN apt update && apt -y install ros-${ROS_DISTRO}-hector-slam
RUN apt update && apt -y install dvtm
RUN apt update && apt -y install vim
RUN apt update && apt -y install ros-${ROS_DISTRO}-async-comm
RUN apt update && apt -y install entr
RUN apt update && apt -y install ros-${ROS_DISTRO}-map-server
RUN apt update && apt -y install ranger
RUN apt update && apt -y install feh

WORKDIR /root
COPY catkin_ws catkin_ws
WORKDIR /root/catkin_ws/src
RUN bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_init_workspace'
WORKDIR /root/catkin_ws
RUN bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make'
RUN bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make install'

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
WORKDIR /root

FROM ros:noetic
SHELL ["/bin/bash", "-c"]

RUN apt update && apt -y install xterm
RUN apt update && apt -y install qt5-default
RUN apt update && apt -y install apturl
RUN apt update && apt -y install python3-pip
RUN apt update && apt -y install ros-${ROS_DISTRO}-tf

COPY scanbot_communicator /app/src/scanbot_communicator
COPY start_communicator.launch /app/src/scanbot_communicator/launch/start_communicator.launch
COPY requirements.txt requirements.txt
RUN pip3 install -r requirements.txt
WORKDIR /app

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && /opt/ros/${ROS_DISTRO}/bin/catkin_make

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source /app/devel/setup.bash" >> /root/.bashrc

#!/bin/bash -ae

# remember to rebuild images with "docker-compose build" if required!

xhost +local:root > /dev/null

# nvidia on-demand on linux:
nvidia_offload="__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia "

# run presets
gazebo_with_gamepad="
roscore_gazebo
rosconsole
gmapping
rviz
gazebo
gamepad
gamepad_conv"
hardware_with_gamepad="
scanbot_hwiface
roscore_hardware
rosconsole
gmapping
rviz
gamepad
gamepad_conv"
rosbag_replay="
roscore_gazebo
rosconsole
gmapping
rviz"

if [ "$1" == "gb" ] ; then
    env $nvidia_offload docker-compose up $gazebo_with_gamepad
elif [ "$1" == "hw" ] ; then
    env $nvidia_offload docker-compose up $hardware_with_gamepad
elif [ "$1" == "rp" ] ; then
    env $nvidia_offload docker-compose up $rosbag_replay
else
    echo "specify container collection to run, available:"
    echo "  gb - gazebo"
    echo "  hw - hardware"
    echo "example: ./start.sh gb"
    exit 1
fi

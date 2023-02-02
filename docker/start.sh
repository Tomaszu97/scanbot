#!/bin/bash -ae

# remember to rebuild images with "docker-compose build" if required!

xhost +local:root > /dev/null

# nvidia on-demand on linux:
nvidia_offload="__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia "

# run presets
gazebo_gmapping="
gmapping
roscore_gazebo
rosconsole
rviz
gazebo
gamepad
gamepad_conv"
gazebo_hector="
hector
roscore_gazebo
rosconsole
rviz
gazebo
gamepad
gamepad_conv"
gazebo_cartographer="
cartographer
roscore_gazebo
rosconsole
rviz
gazebo
gamepad
gamepad_conv"
hardware_gmapping="
scanbot_hwiface
roscore_hardware
rosconsole
rviz
gmapping
gamepad
gamepad_conv"
hardware_hector="
hector
scanbot_hwiface
roscore_hardware
rosconsole
rviz
gamepad
gamepad_conv"
hardware_cartographer="
cartographer
scanbot_hwiface
roscore_hardware
rosconsole
rviz
gamepad
gamepad_conv"
rosbag_replay="
roscore_gazebo
rosconsole
rviz"

if [ "$1" == "gbg" ] || [ "$1" == "gb" ] ; then
    env $nvidia_offload docker-compose up $gazebo_gmapping
elif [ "$1" == "gbh" ] ; then
    env $nvidia_offload docker-compose up $gazebo_hector
elif [ "$1" == "gbc" ] ; then
    env $nvidia_offload docker-compose up $gazebo_cartographer
elif [ "$1" == "hwg" ] || [ "$1" == "hw" ]; then
    env $nvidia_offload docker-compose up $hardware_gmapping
elif [ "$1" == "hwh" ] ; then
    env $nvidia_offload docker-compose up $hardware_hector
elif [ "$1" == "hwc" ] ; then
    env $nvidia_offload docker-compose up $hardware_cartographer
elif [ "$1" == "rp" ] ; then
    env $nvidia_offload docker-compose up $rosbag_replay
else
    echo "specify container collection to run, available:"
    echo "  gb|gbg - gazebo (gmapping)"
    echo "  gbh    - gazebo (hector)"
    echo "  gbc    - gazebo (cartographer)"
    echo "  hw/hwg - hardware (gmapping)"
    echo "  hwh    - hardware (hector)"
    echo "  hwc    - hardware (cartographer)"
    echo "  rp     - replay rosbag"
    echo "example:"
    echo "  ./start.sh gb"
    exit 1
fi

#!/bin/bash -axe

# remember to rebuild images with "docker-compose build" if required!

xhost +local:root

# nvidia on-demand on linux:
nvidia_offload="__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia "

# run presets
gazebo_with_gamepad="roscore rosconsole gmapping rviz gazebo gamepad gamepad_conv"
hw_with_gamepad="roscore rosconsole gmapping rviz gamepad gamepad_conv"

if [ "$1" == "gb" ] ; then
    env $nvidia_offload docker-compose up $gazebo_with_gamepad
elif [ "$1" == "hw" ] ; then
    env $nvidia_offload docker-compose up $hardware_with_gamepad
else
    env $nvidia_offload docker-compose up $@
fi

#!/bin/bash -axe

RATE="${2:-1}"

rosbag play --clock --hz=1 -r $RATE "$1"

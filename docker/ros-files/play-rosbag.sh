#!/bin/bash -axe

rosbag play --clock --hz=1 -r 2 "$@"

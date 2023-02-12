#!/bin/bash -axe
cd records
rosbag record -j \
    /tf \
    /scanbot/scan \

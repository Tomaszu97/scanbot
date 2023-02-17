#!/bin/bash -axe

BAG="$1"
BAG_REPLAY_RATE="$2"
test "$#" == "2"

trap 'pkill -P $$' SIGINT SIGTERM EXIT

# hide frame transforms originally published by slam node
# set this as rosparam due to bug ros/ros_comm#1498 
rosparam set '/tf_remapper/mappings' '[{"old": "map", "new": ""}]'
rosrun tf_remapper_cpp tf_remap &
rosbag play --clock -r "$BAG_REPLAY_RATE" "${BAG}.bag" tf:=tf_old

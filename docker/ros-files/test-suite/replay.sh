#!/bin/bash -ae

BAG="$1"
BAG_REPLAY_RATE="$2"
test "$#" == "2"

replay_cleanup()
{
    echo "replay: cleaning up"
    pkill -P $$ -SIGINT
    exit $1
}

trap "replay_cleanup 0" SIGINT SIGTERM
trap "replay_cleanup -1" ERR SIGHUP

echo "replay: starting tf remapper"
# hide frame transforms originally published by slam node
# set this as rosparam due to bug ros/ros_comm#1498 
rosparam set '/tf_remapper/mappings' '[{"old": "map", "new": ""}]'
rosrun tf_remapper_cpp tf_remap &

echo "replay: playing rosbag"
rosbag play --clock -r "$BAG_REPLAY_RATE" "${BAG}.bag" tf:=tf_old
replay_cleanup 0

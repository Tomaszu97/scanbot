#!/bin/bash -axe

BAG="$1"
BAG_REPLAY_RATE="$2"
ALGORITHM="$3"
test "$#" == "3"

trap 'trap - SIGTERM && kill 0' SIGINT SIGTERM EXIT
./"${ALGORITHM}.sh" &
rosbag play -r "$BAG_REPLAY_RATE" "${BAG}.bag"

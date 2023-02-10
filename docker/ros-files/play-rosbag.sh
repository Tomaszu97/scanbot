#!/bin/bash -axe

BAG_FILENAME="$1"
RATE="${2:-1}"
ALGORITHM_SCRIPT="$3"
test -n "$BAG_FILENAME"
test -n "$ALGORITHM_SCRIPT"

"$ALGORITHM_SCRIPT" &
ALGORITHM_SCRIPT_PID=$!
rosbag play --clock --hz=1 -r "$RATE" "$BAG_FILENAME"
echo press enter to stop
read

pkill -P $ALGORITHM_SCRIPT_PID

echo algorithm killed

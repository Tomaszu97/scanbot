#!/bin/bash -axe

ALGORITHM="$1"
BAG="$2"
BAG_REPLAY_RATE="$3"
TEST_PARAM_NAME="$4"
TEST_PARAM_MIN="$5"
TEST_PARAM_STEP="$6"
TEST_PARAM_MAX="$7"
test "$#" == "7"

for curr_param_value in $(seq "$TEST_PARAM_MIN" "$TEST_PARAM_STEP" "$TEST_PARAM_MAX")
do
    : "running test for param $curr_param_value"
    : "starting algorithm $ALGORITHM"
    env "${ALGORITHM}_${TEST_PARAM_NAME}=${curr_param_value}" ./run-algorithm.sh "${ALGORITHM}" &
    algo_pid="$!"

    : "replaying bag $BAG"
    ./replay.sh "$BAG" "$BAG_REPLAY_RATE"

    : "saving map as image"
    curr_dirname="test/${ALGORITHM}/${BAG}_x${BAG_REPLAY_RATE}/${TEST_PARAM_NAME}/"
    mkdir -p "$curr_dirname"
    ./save-map.sh "${curr_dirname}/${curr_param_value}"

    : "killing algorithm"
    while kill -HUP "$algo_pid"
    do
        sleep 1
    done
done

rm "${ALGORITHM}.testoverrides"

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

    echo "${ALGORITHM}_${TEST_PARAM_NAME}=${curr_param_value}" > "${ALGORITHM}.testoverrides"
    ./replay-algo.sh "$BAG" "$BAG_REPLAY_RATE" "$ALGORITHM"

    curr_dirname="test/${ALGORITHM}/${BAG}_x${BAG_REPLAY_RATE}/${TEST_PARAM_NAME}/"
    mkdir -p "$curr_dirname"
    ./save-map.sh "${curr_dirname}/${curr_param_value}"
done

rm "${ALGORITHM}.testoverrides"

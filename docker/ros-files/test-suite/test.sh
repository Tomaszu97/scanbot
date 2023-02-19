#!/bin/bash -ae

ALGORITHM="$1"
BAG="$2"
BAG_REPLAY_RATE="$3"
TEST_PARAM_NAME="$4"
TEST_PARAM_MIN="$5"
TEST_PARAM_STEP="$6"
TEST_PARAM_MAX="$7"
test "$#" == "7"

cleanup()
{
    echo "test: cleaning up"
    pkill -P $$ -SIGINT
    exit $1
}

trap "cleanup 0" SIGINT SIGTERM
trap "cleanup -1" ERR SIGHUP

for curr_param_value in $(seq "$TEST_PARAM_MIN" "$TEST_PARAM_STEP" "$TEST_PARAM_MAX")
do
    echo "test: running - algorithm=${ALGORITHM}, param ${TEST_PARAM_NAME}=${curr_param_value}, bag=${BAG}"
    echo "test: starting algorithm"
    env "${ALGORITHM}_${TEST_PARAM_NAME}=${curr_param_value}" ./run-algorithm.sh "${ALGORITHM}" &
    algo_pid="$!"

    echo "test: replaying bag"
    ./replay.sh "$BAG" "$BAG_REPLAY_RATE"

    echo "test: checking if algorithm is still running"
    if kill -0 "$algo_pid"
    then
        curr_dirname="test/${ALGORITHM}/${BAG}_x${BAG_REPLAY_RATE}/${TEST_PARAM_NAME}/"
        echo "test: saving map as image, dir=${curr_dirname}"
        mkdir -p "$curr_dirname"
        ./save-map.sh "${curr_dirname}/${curr_param_value}"
    else
        echo "test: algorithm died - skip map fetching"
        echo "test: cleanup dead nodes from ros core"
        yes | rosnode cleanup
    fi

    echo "test: killing algorithm"
    kill -0 "$algo_pid" && kill -SIGINT "$algo_pid"
done

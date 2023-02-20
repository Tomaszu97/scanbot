#!/bin/bash -a

BAG="$1"
BAG_REPLAY_RATE="$2"
test "$#" == "2" || { echo 'check parameters' ; exit 1 ; }

cleanup()
{
    echo "multitest2: cleaning up"
    pkill -P $$ -SIGINT
    exit $1
}

trap "cleanup 0" SIGINT SIGTERM
trap "cleanup -1" SIGHUP

echo "multitest2: running multitest for $BAG"

echo "multitest2: trying algorithm: hector"
./test.sh hector "$BAG" "$BAG_REPLAY_RATE" _map_resolution                           0.01    0.005   0.09
./test.sh hector "$BAG" "$BAG_REPLAY_RATE" _map_update_distance_thresh               0.1     0.1     1.7
./test.sh hector "$BAG" "$BAG_REPLAY_RATE" _map_update_angle_thresh                  0.1     0.1     1.7
./test.sh hector "$BAG" "$BAG_REPLAY_RATE" _map_multi_res_levels                     1       0.25    5
./test.sh hector "$BAG" "$BAG_REPLAY_RATE" _update_factor_free                       0.2     0.05    1.0
./test.sh hector "$BAG" "$BAG_REPLAY_RATE" _update_factor_occupied                   0.1     0.05    0.9

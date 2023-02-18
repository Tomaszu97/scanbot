#!/bin/bash -ax

BAG="$1"
BAG_REPLAY_RATE="$2"
test "$#" == "2" || exit 1

: "running multitest for $BAG"

: "trying algorithm: gmapping"
./test.sh gmapping "$BAG" "$BAG_REPLAY_RATE" _map_update_interval                    0.1     2.0     8.1
./test.sh gmapping "$BAG" "$BAG_REPLAY_RATE" _sigma                                  0.01    0.02    0.09
./test.sh gmapping "$BAG" "$BAG_REPLAY_RATE" _kernelSize                             1       1       5
./test.sh gmapping "$BAG" "$BAG_REPLAY_RATE" _lstep                                  0.01    0.02    0.09
./test.sh gmapping "$BAG" "$BAG_REPLAY_RATE" _astep                                  0.01    0.02    0.09
./test.sh gmapping "$BAG" "$BAG_REPLAY_RATE" _iterations                             1       2       9
./test.sh gmapping "$BAG" "$BAG_REPLAY_RATE" _lsigma                                 0.025   0.025   0.125
./test.sh gmapping "$BAG" "$BAG_REPLAY_RATE" _ogain                                  1.0     1.0     5.0
./test.sh gmapping "$BAG" "$BAG_REPLAY_RATE" _lskip                                  0       1       4
./test.sh gmapping "$BAG" "$BAG_REPLAY_RATE" _minimumScore                           0.0     50.0    200.0
./test.sh gmapping "$BAG" "$BAG_REPLAY_RATE" _srr                                    0.01    0.2     0.81
./test.sh gmapping "$BAG" "$BAG_REPLAY_RATE" _srt                                    0.01    0.4     1.61
./test.sh gmapping "$BAG" "$BAG_REPLAY_RATE" _str                                    0.01    0.2     0.81
./test.sh gmapping "$BAG" "$BAG_REPLAY_RATE" _stt                                    0.01    0.4     1.61
./test.sh gmapping "$BAG" "$BAG_REPLAY_RATE" _linearUpdate                           0.1     0.4     1.7
./test.sh gmapping "$BAG" "$BAG_REPLAY_RATE" _angularUpdate                          0.1     0.2     0.9
./test.sh gmapping "$BAG" "$BAG_REPLAY_RATE" _temporalUpdate                         0.1     0.2     0.9
./test.sh gmapping "$BAG" "$BAG_REPLAY_RATE" _resampleThreshold                      0.1     0.2     0.9
./test.sh gmapping "$BAG" "$BAG_REPLAY_RATE" _particles                              10      20      90
./test.sh gmapping "$BAG" "$BAG_REPLAY_RATE" _delta                                  0.01    0.04    0.17
./test.sh gmapping "$BAG" "$BAG_REPLAY_RATE" _llsamplerange                          0.005   0.005   0.025
./test.sh gmapping "$BAG" "$BAG_REPLAY_RATE" _llsamplestep                           0.005   0.005   0.025
./test.sh gmapping "$BAG" "$BAG_REPLAY_RATE" _lasamplerange                          0.0025  0.0025  0.0125
./test.sh gmapping "$BAG" "$BAG_REPLAY_RATE" _lasamplestep                           0.0025  0.0025  0.0125
./test.sh gmapping "$BAG" "$BAG_REPLAY_RATE" _occ_thresh                             0.05    0.1     0.45

: "trying algorithm: hector"
./test.sh hector "$BAG" "$BAG_REPLAY_RATE" _map_resolution                           0.01    0.02    0.09
./test.sh hector "$BAG" "$BAG_REPLAY_RATE" _map_update_distance_thresh               0.1     0.4     1.7
./test.sh hector "$BAG" "$BAG_REPLAY_RATE" _map_update_angle_thresh                  0.1     0.4     1.7
./test.sh hector "$BAG" "$BAG_REPLAY_RATE" _map_multi_res_levels                     1       1       5
./test.sh hector "$BAG" "$BAG_REPLAY_RATE" _update_factor_free                       0.1     0.1     0.5
./test.sh hector "$BAG" "$BAG_REPLAY_RATE" _update_factor_occupied                   0.1     0.2     0.9

: "trying algorithm: karto"
./test.sh karto "$BAG" "$BAG_REPLAY_RATE" _map_update_interval                       0.1     2.0     8.1
./test.sh karto "$BAG" "$BAG_REPLAY_RATE" _delta                                     0.01    0.04    0.17
./test.sh karto "$BAG" "$BAG_REPLAY_RATE" _minimum_travel_distance                   0.1     0.4     1.7
./test.sh karto "$BAG" "$BAG_REPLAY_RATE" _minimum_travel_heading                    0.05    0.1     0.45
./test.sh karto "$BAG" "$BAG_REPLAY_RATE" _scan_buffer_size                          10      20      90
./test.sh karto "$BAG" "$BAG_REPLAY_RATE" _link_match_minimum_response_fine          0.1     0.4     1.7
./test.sh karto "$BAG" "$BAG_REPLAY_RATE" _link_scan_maximum_distance                1.0     4.0     17.0
./test.sh karto "$BAG" "$BAG_REPLAY_RATE" _loop_search_maximum_distance              1.0     2.0     9.0
./test.sh karto "$BAG" "$BAG_REPLAY_RATE" _loop_match_minimum_chain_size             1       4       17
./test.sh karto "$BAG" "$BAG_REPLAY_RATE" _loop_match_maximum_variance_coarse        0.2     0.2     1.0
./test.sh karto "$BAG" "$BAG_REPLAY_RATE" _loop_match_minimum_response_coarse        0.1     0.4     1.7
./test.sh karto "$BAG" "$BAG_REPLAY_RATE" _loop_match_minimum_response_fine          0.1     0.4     1.7
./test.sh karto "$BAG" "$BAG_REPLAY_RATE" _correlation_search_space_dimension        0.1     0.4     1.7
./test.sh karto "$BAG" "$BAG_REPLAY_RATE" _correlation_search_space_resolution       0.005   0.005   0.025
./test.sh karto "$BAG" "$BAG_REPLAY_RATE" _correlation_search_space_smear_deviation  0.01    0.02    0.09
./test.sh karto "$BAG" "$BAG_REPLAY_RATE" _correlation_loop_search_space_dimension   1.0     4.0     17.0
./test.sh karto "$BAG" "$BAG_REPLAY_RATE" _loop_search_space_resolution              0.01    0.04    0.17
./test.sh karto "$BAG" "$BAG_REPLAY_RATE" _loop_search_space_smear_deviation         0.01    0.01    0.05
./test.sh karto "$BAG" "$BAG_REPLAY_RATE" _distance_variance_penalty                 0.4     0.05    0.6
./test.sh karto "$BAG" "$BAG_REPLAY_RATE" _angle_variance_penalty                    0.25    0.05    0.45
./test.sh karto "$BAG" "$BAG_REPLAY_RATE" _fine_search_angle_offset                  0.001   0.001   0.005
./test.sh karto "$BAG" "$BAG_REPLAY_RATE" _coarse_search_angle_offset                0.1     0.1     0.5
./test.sh karto "$BAG" "$BAG_REPLAY_RATE" _coarse_angle_resolution                   0.1     0.1     0.5
./test.sh karto "$BAG" "$BAG_REPLAY_RATE" _minimum_angle_penalty                     0.1     0.4     1.7
./test.sh karto "$BAG" "$BAG_REPLAY_RATE" _minimum_distance_penalty                  0.1     0.2     0.9

#!/bin/bash -axe
PREFIX="$1"
test -n "$1"
rosrun map_server map_saver -f "$PREFIX"

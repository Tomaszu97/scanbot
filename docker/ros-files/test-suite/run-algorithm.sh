#!/bin/bash -ae

ALGORITHM="$1"
test "$#" == "1" || { echo "run-algorithm: check parameters" ; exit 1 ; }

echo "run-algorithm: sourcing parameters"
test -e "${ALGORITHM}.overrides" && source "${ALGORITHM}.overrides"
test -e "${ALGORITHM}.defaults" && source "${ALGORITHM}.defaults"

echo "run-algorithm: exec algorithm"
exec "./${ALGORITHM}.run"

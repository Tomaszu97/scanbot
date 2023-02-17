#!/bin/bash -axe

ALGORITHM="$1"
test "$#" == "1"

test -e "${ALGORITHM}.overrides" && source "${ALGORITHM}.overrides"
test -e "${ALGORITHM}.overrides" && source "${ALGORITHM}.defaults"

"./${ALGORITHM}.run"

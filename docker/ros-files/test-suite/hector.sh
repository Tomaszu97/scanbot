#!/bin/bash -axe
source hector.defaults
test -e hector.overrides && source hector.overrides
test -e hector.testoverrides && source hector.testoverrides
./hector.run

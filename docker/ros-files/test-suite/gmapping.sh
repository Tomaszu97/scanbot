#!/bin/bash -axe
source gmapping.defaults
test -e gmapping.overrides && source gmapping.overrides
test -e gmapping.testoverrides && source gmapping.testoverrides
./gmapping.run

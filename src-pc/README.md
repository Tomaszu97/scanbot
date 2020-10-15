# PC Application notes
---
# IMPORTANT - WILL NOT WORK OTHERWISE:
Install pyqtgraph from repository.
Version 0.11.0 doesn't have clear() implemented
in 3d graphs.

# MANUAL CALIBRATION:
SET_MAG_CAL:x,y,z,a1,a2,a3,a4,mul#
Above command is used to write calibration data
to EEPROM. Put negative offset values as x y and z. 
For angle calibration a1 a2 a3 and a4 are used. 
They should be about 0, 90, 180 and 270 (0-360).
Set mul to 1 for normal operation, -1 for reverse
(not recommended).
#!/bin/bash -axe

#usage
#./convert_sdf_to_urdf.sh model.sdf model.urdf

source catkin_ws/devel/setup.bash
MESH_WORKSPACE_PATH=. rosrun pysdf sdf2urdf.py $1 $2

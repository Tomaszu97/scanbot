#!/bin/bash -axe
xhost +local:root
rsync --archive --delete ../src-pc/scanbot_communicator/ communicator/scanbot_communicator/
rsync --archive --delete ../src-pc/scanbot_communicator/res/rviz-config.rviz  rviz/
docker-compose build && docker-compose up

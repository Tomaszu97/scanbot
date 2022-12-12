#!/bin/bash -axe
xhost +local:root
docker-compose build && docker-compose up

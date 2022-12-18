#!/bin/bash -axe

# remember to rebuild images with "docker-compose build" if required!

xhost +local:root

# nvidia on-demand on linux:
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
docker-compose up $@

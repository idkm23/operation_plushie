#!/bin/bash
echo "HOLY CRAP YAY" 1>&2
source ~/indigo/devel/setup.bash
export ROS_MASTER_URI=http://baxter.local:11311
exec "$@"

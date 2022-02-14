#!/bin/bash

export PATH=$HOME/packages/PAL-Firmware-v1.2-Intel-CPU/installations/dreamvu_ws/bin:$PATH

source /opt/ros/foxy/setup.bash
source ../../install/local_setup.bash

## ros2 run pal_camera capture
ros2 launch pal_camera pal_camera.launch.py

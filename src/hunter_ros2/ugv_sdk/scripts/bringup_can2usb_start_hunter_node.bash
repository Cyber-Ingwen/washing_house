#!/bin/bash

# bring up can interface
sudo ip link set can0 up type can bitrate 500000
source install/setup.bash
ros2 launch hunter_base hunter_base.launch.py
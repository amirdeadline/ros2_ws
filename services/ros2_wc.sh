#!/bin/bash
cd ~/lawnbot/ && source install/setup.bash
ros2 launch weed_control weed_control_launch.py

#!/bin/bash

source install/setup.bash
# Run the lidar node
ros2 launch innovusion ivu_pc2.py device_ip:=172.168.1.10
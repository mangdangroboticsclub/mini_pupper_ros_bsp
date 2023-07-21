#!/bin/bash

cd ~/mini_pupper_ros_bsp/tests/test_ws
colcon build --symlink-install
source install/setup.bash
ros2 run test_api test_display

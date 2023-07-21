#!/bin/bash

source ~/ros_ws/install/setup.bash
ros2 run mini_pupper_driver display_interface --ros-args --remap /mini_pupper_lcd/image_raw:=/image_raw

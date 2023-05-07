#!/bin/bash

source  ~/mini-pupper-release
if [ "$MACHINE" == "x86_64" ]
then
ACTIVE="rc-local battery_monitor joy_node lidar servo_interface display_interface v4l2_camera robot"
INACTIVE=""
if [ "$HARDWARE" == "mini_pupper_2" ]
then
ACTIVE=$ACTIVE" esp32-proxy imu"
fi
else
ACTIVE="rc-local battery_monitor joy_node lidar servo_interface display_interface v4l2_camera robot"
INACTIVE=""
if [ "$HARDWARE" == "mini_pupper_2" ]
then
ACTIVE=$ACTIVE" esp32-proxy imu"
fi
fi

# Give services time to start
sleep 10

for SERVICE in $ACTIVE; do
    STATE=$(sudo systemctl show $SERVICE | grep ActiveState | awk -F'=' '{print $2}')
    if [ "$STATE" == "active" ]
    then
	echo $SERVICE is OK
    else
	echo $SERVICE is NOT OK
    fi
done

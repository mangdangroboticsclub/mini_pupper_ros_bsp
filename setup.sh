#!/bin/bash
IS_RELEASE=No

set -e
echo "setup.sh started at $(date)"

### Get directory where this script is installed
BASEDIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# check Ubuntu version
source /etc/os-release

if [[ $UBUNTU_CODENAME != 'jammy' ]]
then
    echo "Ubuntu 22.04 LTS (Jammy Jellyfish) is required"
    echo "You are using $VERSION"
    exit 1
fi

cd ~
if [ ! -d ~/mini_pupper_bsp ]
then
[[ "$1" == "v1" ]] && git clone https://github.com/mangdangroboticsclub/mini_pupper_bsp.git mini_pupper_bsp
[[ "$1" == "v2" ]] && git clone https://github.com/mangdangroboticsclub/mini_pupper_2_bsp.git mini_pupper_bsp
fi

# Install Mini Pupper BSP
~/mini_pupper_bsp/install.sh

# Install Mini Pupper ROS BSP
~/mini_pupper_ros_bsp/install.sh $1

echo "setup.sh finished at $(date)"
source  ~/mini-pupper-release
if [ "$MACHINE" == "x86_64" ]
then
    sudo systemctl start rc-local
if [ "$HARDWARE" == "mini_pupper_2" ]
then
    sudo systemctl start esp32-proxy &
    sudo systemctl start battery_monitor &
else
    sudo systemctl start battery_monitor
fi
    sudo systemctl start robot
    sudo systemctl start servo_interface
    sudo systemctl start display_interface
    sudo systemctl start joystick
    sudo systemctl start joy_node
    sudo systemctl start restart_joy
    sudo systemctl start lidar
    sudo systemctl start imu
    sudo systemctl start v4l2_camera
else
    sudo reboot
fi

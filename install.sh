#!/bin/bash

# check Ubuntu version
source /etc/os-release

if [[ $UBUNTU_CODENAME != 'jammy' ]]
then
    echo "Ubuntu 22.04.1 LTS (Jammy Jellyfish) is required"
    echo "You are using $VERSION"
    exit 1
fi

### Get directory where this script is installed
BASEDIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

### Append to release file
echo ROSBSP_VERSION=\"$(cd ~/mini_pupper_ros_bsp; ~/mini_pupper_bsp/get-version.sh)\" >> ~/mini-pupper-release

source  ~/mini-pupper-release
if [ "$IS_RELEASE" == "YES" ]
then
    cd $BASEDIR
    TAG_COMMIT=$(git rev-list --abbrev-commit --tags --max-count=1)
    TAG=$(git describe --abbrev=0 --tags ${TAG_COMMIT} 2>/dev/null || true)
    if [ "v$ROSBSP_VERSION" != "$TAG" ]
    then
        sed -i "s/IS_RELEASE=YES/IS_RELEASE=NO/" ~/mini-pupper-release
    fi
fi

### Install ROS2
cd ~
git clone https://github.com/Tiryoh/ros2_setup_scripts_ubuntu.git
./ros2_setup_scripts_ubuntu/run.sh

source /opt/ros/humble/setup.bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
git clone https://github.com/mangdangroboticsclub/mini_pupper_ros_bsp.git

cd ~/ros_ws
rosdep update && rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent --skip-keys micro_ros_agent
sudo pip install setuptools==58.2.0 # suppress colcon build warning
colcon build --executor sequential --symlink-install

# Install servo_interface
cd ~/mini_pupper_ros_bsp/services
sudo ln -s $(realpath .)/servo_interface.service /etc/systemd/system/

# Install display_interface
cd ~/mini_pupper_ros_bsp/services
sudo ln -s $(realpath .)/display_interface.service /etc/systemd/system/

# Install Joystick
sudo adduser ubuntu input
sudo pip install ds4drv
sudo wget https://raw.githubusercontent.com/chrippa/ds4drv/master/udev/50-ds4drv.rules -O /etc/udev/rules.d/50-ds4drv.rules
cd ~/mini_pupper_ros_bsp/services
sudo ln -s $(realpath .)/joystick.service /etc/systemd/system/
sudo ln -s $(realpath .)/joy_node.service /etc/systemd/system/
sudo ln -s $(realpath .)/restart_joy.service /etc/systemd/system/

# Install robot service
# IMU and Lidar depnd on it
cd ~/mini_pupper_ros_bsp/services
sudo ln -s $(realpath .)/robot.service /etc/systemd/system/

# Install Lidar
source /opt/ros/humble/setup.bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git src/ldlidar
colcon build --executor sequential --symlink-install
cd ~/mini_pupper_ros_bsp/services
sudo ln -s $(realpath .)/lidar.service /etc/systemd/system/

# Install IMU
if [[ "$1" == "v2" ]]
then
cd ~/mini_pupper_ros_bsp/services
sudo ln -s $(realpath .)/imu.service /etc/systemd/system/
sudo apt install -y ros-humble-imu-tools
fi

# Install Camera
MACHINE=$(uname -m)
if [ "$MACHINE" != "x86_64" ]
# install script will break virtual installation
then
~/mini_pupper_bsp/RPiCamera/install.sh
fi
cd ~/mini_pupper_ros_bsp/services
sudo ln -s $(realpath .)/v4l2_camera.service /etc/systemd/system/
sudo apt install -y ros-humble-v4l2-camera ros-humble-image-transport-plugins
mkdir -p ~/.ros/camera_info/
# copy defaultcamera calibration file. This should be replaced by a camera spefici calibration file
cp $BASEDIR/services/mmal_service_16.1.yaml ~/.ros/camera_info/


# Cyclon DDS
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc

# enable services
sudo systemctl daemon-reload
sudo systemctl enable robot
sudo systemctl enable servo_interface
sudo systemctl enable display_interface
sudo systemctl enable joystick
sudo systemctl enable joy_node
sudo systemctl enable restart_joy
sudo systemctl enable lidar
sudo systemctl enable imu
sudo systemctl enable v4l2_camera

#!/bin/bash

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
[[ "$1" == "v1" ]] && git clone https://github.com/mangdangroboticsclub/mini_pupper_bsp.git mini_pupper_bsp
[[ "$1" == "v2" ]] && git clone https://github.com/mangdangroboticsclub/mini_pupper_2_bsp.git mini_pupper_bsp
#TODO Change URL after testing
[[ -d ~/mini_pupper_ros_bsp ]] || git -b for_review https://github.com/hdumcke/mini_pupper_ros_bsp.git

# Install Mini Pupper BSP
MACHINE=$(uname -m)
if [ "$MACHINE" == "x86_64" ]
then
    # We are installing in a virtual machine
    sudo apt-get update
    sudo apt-get -y install python3 python3-pip python-is-python3 python3-venv python3-virtualenv
    #TODO re-enable version
    #sudo PBR_VERSION=$(cd ~/mini_pupper_bsp; ./get-version.sh) pip install ~/mini_pupper_bsp/mock_api
    sudo DUMMY=$(cd ~/mini_pupper_bsp; ./get-version.sh) pip install ~/mini_pupper_bsp/mock_api
else
    #TODO re-enable version
    sed -i "s/PBR_VERSION/DUMMY/" ~/mini_pupper_bsp/install.sh
    ~/mini_pupper_bsp/install.sh
fi

# Install Mini Pupper ROS BSP
~/mini_pupper_ros_bsp/install.sh

echo "setup.sh finished at $(date)"
sudo reboot

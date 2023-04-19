# ISSUES

## IMU

This repo currenty deploys an IMU node that reads IMU data from the physical IMU, applies some calibration and corrections and publised on the topic /imu/data_raw

It also deploys a complimentary filter that converts raw data and publish /imu/data Once of the conversion is that /imu/data contains quaternions where /imu/data_raw does not.

As there are many different filters for IMU data available this filter is an opinited choice. The intent was to make the launch scrip customizable to choose between complementary and madgwick filter but this work is not complete.

Choices are:

- leave as is for convenience. If application needs a different filter application has to install a different filter
- remove the filter from this repo and let every application install the IMU filter if required
- spend some energy to make the launch script customizable in which case the application has to deploy a config file which chooses the kind of filter

The Lidar node publish as TF the identity. This is technical incorrect. The IMU is ohysically mounted displaced from the origin of the robot and rotated. The currection is currently done in the IMU node so that the axis show up correctly. A better implementation would be to provide a correct rotation in the TF, this should be considered for later work but should not have any implication for the application consuming IMU data

## LIDAR

LIDAR connection (LD06) needs to be tested on 

mini pupper v2 with CM4 directly connected to the board
mini pupper v1 and v2 with RPi 4, Lidar connected to USB
Automatic discovery???

## Display driver

Current display driver is broken and terminaates with Python error

Display driver must not bypass MangDang.mini_pupper.display, we have to investigate if the Python API requires additinal capabilities like passing im memery iages

Need test scripts

## Servo driver

Need test scripts

## Integration with CHAMP

I believe that this work should work with only little modification with the CHAMP project in mini_pupper_ros repo. The change will be mainly in launch scripts to remove all nodes that are already running on mini pupper

There will be some services that are better run directly on the robot itself like for example teleop-joy. The best way to solve this would be a setup script in mini_pupper_ros repo, I can help with this.

There might be other issues we might discover during testing

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

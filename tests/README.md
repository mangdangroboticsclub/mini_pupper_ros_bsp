# Mini Pupper ROS Testing

## Test Services

To test if all services are running issue the following connads:

```sh
sudo systemctl status robot
sudo systemctl status imu
sudo systemctl status joy_node
sudo systemctl status lidar
sudo systemctl status servo_interface
sudo systemctl status display_interface
sudo systemctl status v4l2_camera
```

## Test ROS Nodes

check that the ROS nodes are running:

```sh
ros2 node list
```

You should see the followig list:

```sh
/base_link_to_base_laser_ld06
/base_link_to_imu
/imu_complementary_filter
/joy_node
/mini_pupper_imu_driver_node
/servo_interface
/v4l2_camera
```

## Test ROS Topics

check that the ROS topics are available:

```sh
ros2 topic list
```

You should see the followig list:

```sh
/camera_info
/image_raw
/image_raw/compressed
/image_raw/compressedDepth
/image_raw/theora
/imu/data
/imu/data_raw
/joint_group_effort_controller/joint_trajectory
/joy
/joy/set_feedback
/mini_pupper_lcd/image_raw
/parameter_events
/rosout
/tf
/tf_static
```

## Echo ROS Topics that are Produced

For each topic that generates output echo the topic

```sh
ros2 topic echo <topic name>
```

## Test Joystick

To test the joystick driver connect with your joystick and issue:

```sh
ros2 topic echo /joy
```

You should see joystick actions in the ROS topic

## Move Servos

To test the servo driver:

```sh
cd ~/mini_pupper_ros_bsp/tests/test_ws
colcon build --symlink-install
source install/setup.bash
ros2 run test_api test_servos
```

## Test LCD

To test the display driver:

```sh
cd ~/mini_pupper_ros_bsp/tests/test_ws
colcon build --symlink-install
source install/setup.bash
ros2 run test_api test_display
```

The LCD display should show blinking eyes for 2 seconds.

## Test Video Stream

Run the following command on your PC and select topic /image_raw

```sh
ros2 run rqt_image_view rqt_image_view
```

To display the video stream on the LDC use the following commands on mini pupper:

```sh
sudo systemctl stop display_interface
source ~/ros_ws/install/setup.bash
ros2 run mini_pupper_driver display_interface --ros-args --remap /mini_pupper_lcd/image_raw:=/image_raw
```

** This test does not work as expected. LCD updayed is too slow **

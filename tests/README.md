# Mini Pupper ROS Testing

## Test Services

To test if all services are running issue the following connads:

```sh
sudo systemctl status imu
sudo systemctl status joystick
sudo systemctl status lidar
sudo systemctl status mini_pupper_driver
sudo systemctl status restart_joy
sudo systemctl status v4l2_camera
```

## Test ROS Nodes

check that the ROS nodes are running:

```sh
ros2 node list
```

## Test ROS Topics

check that the ROS topics are available:

```sh
ros2 topic list
```

## Echo ROS Topics that are Produces

For each topic that generates output echo the topic

```sh
ros2 topic echo <topic name>
```

## Test Joystick

To test the joystick driver:

```sh

```

## Move Servos

To test the servo driver:

```sh

```

## Test LCD

To test the display driver:

```sh

```

## Test Video Stream

Run the following command on your PC and select topic /image_raw

```sh
ros2 run rqt_image_view rqt_image_view
```

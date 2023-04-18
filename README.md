# Mini Pupper ROS  Base Support Package

## installation

### Flash Ubuntu preinstalled image to the SD card.

* Download ubuntu-22.04.2-preinstalled-desktop-arm64+raspi.img.xz from the official website https://ubuntu.com/download/raspberry-pi

Flash a SD card with the tool of your choice

Clone this repository on the PC where you have created your SD card. Make sure the SD card is mounted. Run

```
prepare_sd.py
```

And answer the questions. At the end eject your SD card.

If you are using Windows, run prepare_sd.bat instead of prepare_sd.py.

### First Boot of Mini Pupper

Take the SD card you have prepared, stick it into Mini Pupper, boot Mini Pupper and wait until the IP address is shown on the LCD.

## ROS Nodes

### Mini Pupper Driver

### IMU

### Lidar

### Joystick

### Raspberry Pi V412 Camera

## ROS Domain ID

By default the ROS_DOMAIN_ID is set to 0

To change the domain ID edit /home/ubuntu/mini_pupper_ros_bsp/services/ROS_DOMAIN_ID and reboot Mini Pupper

## ROS DDS

Mini Pupper uses Cyclone-DDS

## Testing

Consult testing/README.md

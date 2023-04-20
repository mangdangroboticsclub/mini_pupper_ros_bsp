#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2022-2023 MangDang
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# @Author  : Yunlong Feng

from PIL import Image as Img
import rclpy
from rclpy.node import Node
import cv2
import tempfile
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from MangDang.mini_pupper.display import Display


class DisplayNode(Node):
    def __init__(self):
        super().__init__('display_interface')
        self.get_logger().info("Initializing display interface")
        self.bridge = CvBridge()
        self.disp = Display()
        self.disp.show_ip()
        self.sub = self.create_subscription(
            Image,
            'mini_pupper_lcd/image_raw',
            self.callback,
            10)

    def callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        image = Img.fromarray(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))
        with tempfile.NamedTemporaryFile() as temp:
            image.save(temp.name + '.png')
            self.disp.show_image(temp.name + '.png')


def main(args=None):
    rclpy.init(args=args)

    display_node = DisplayNode()

    rclpy.spin(display_node)

    display_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

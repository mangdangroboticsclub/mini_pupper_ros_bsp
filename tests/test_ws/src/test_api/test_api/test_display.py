import cv2
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge


class OpenCVDemo(Node):

    def __init__(self):
        super().__init__('test_display')
        self.pub_img = self.create_publisher(Image,
                                             "/mini_pupper_lcd/image_raw", 10)

        test_images = ["/var/lib/mini_pupper_bsp/trot.png",
                       "/var/lib/mini_pupper_bsp/rest.png"]

        for i in range(20):
            cv_img = cv2.imread(test_images[i % 2], 0)
            self.cvb = CvBridge()
            out_msg = self.cvb.cv2_to_imgmsg(cv_img)
            out_msg.header.frame_id = 'test_image'
            out_msg.encoding = 'bgr8'
            self.pub_img.publish(out_msg)
            time.sleep(0.1)

        quit()


def main(args=None):
    rclpy.init(args=args)

    demo = OpenCVDemo()

    rclpy.spin(demo)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

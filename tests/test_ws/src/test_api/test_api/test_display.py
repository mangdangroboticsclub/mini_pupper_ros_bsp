import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

class OpenCVDemo(Node):

    def __init__(self):
        super().__init__('test_display')
        self.pub_img = self.create_publisher(Image,
                                             "/mini_pupper_lcd/image_raw", 10)

        cv_img = cv2.imread('/home/ubuntu/mini_pupper_ros_bsp/tests/test.png', 0)
        self.cvb = CvBridge()
        out_msg = self.cvb.cv2_to_imgmsg(cv_img)
        out_msg.header.frame_id = 'test_image'
        out_msg.encoding = 'bgr8'
        self.pub_img.publish(out_msg)


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

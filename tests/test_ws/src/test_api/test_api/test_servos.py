import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ServoDemo(Node):

    def __init__(self):
        super().__init__('test_servos')
        self.init_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.positions = self.declare_parameter("positions", self.init_positions).value
        self.pub_trajectories = self.create_publisher(JointTrajectory, "/joint_group_effort_controller/joint_trajectory", 10)
        self.get_logger().info("Read servo positions: "f"{self.positions}")
        traj_msg = JointTrajectory()
        traj_msg.points.append(JointTrajectoryPoint())
        traj_msg.points[0].positions = self.positions
        self.pub_trajectories.publish(traj_msg)


def main(args=None):
    rclpy.init(args=args)

    demo = ServoDemo()

    rclpy.spin(demo)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

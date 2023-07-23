import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ServoTest(Node):

    def __init__(self):
        super().__init__('test_display')
        self.pub = self.create_publisher(JointTrajectory,
                                         "/joint_group_effort_controller/joint_trajectory",
                                         10)
        self.dt = 0.05  # 20 Hz
        self.neutral_position = [[-0.00713329, 0.00713329, -0.00713329, 0.00713329],
                                 [0.99422964, 0.99422964, 0.99422964, 0.99422964],
                                 [-0.77346141, -0.77346141, -0.77346141, -0.77346141]]
        self.ticks = 0
        self.max_ticks = 20
        self.max_angle = 5.
        self.direction = 1
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def timer_callback(self):
        deviation = self.direction * self.ticks * self.max_angle * self.dt * 0.0174533
        positions = []
        for j in range(4):
            for i in range(3):
                pos = self.neutral_position[i][j]
                if i == 2:
                    pos += deviation
                positions.append(pos)
        self.ticks += self.direction
        if self.ticks == self.max_ticks:
            self.direction = -1
        if self.ticks == -self.max_ticks:
            self.direction = 1
        goal_positions = positions
        msg = JointTrajectory()
        point = JointTrajectoryPoint()
        point.positions = goal_positions
        msg.points.append(point)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    servo_test = ServoTest()

    rclpy.spin(servo_test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    servo_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

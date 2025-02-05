import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SteeringController(Node):
    def __init__(self):
        super().__init__('steering_controller')
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_position_controller/joint_trajectory',
            10
        )
        self.timer = self.create_timer(2.0, self.send_steering_command)  # Send command every 2 seconds
        self.angle = 0.0  # Initial turn angle

    def send_steering_command(self):
        msg = JointTrajectory()
        msg.joint_names = ['front_left_steering', 'front_right_steering']

        point = JointTrajectoryPoint()
        point.positions = [self.angle, self.angle]  # Set steering angle
        point.time_from_start.sec = 1  # 1 second transition

        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info(f"Steering set to {self.angle:.2f} radians")

        # Alternate left and right turn
        self.angle = -self.angle


def main(args=None):
    rclpy.init(args=args)
    node = SteeringController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

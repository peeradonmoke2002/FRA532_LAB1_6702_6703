#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

class SteeringAnglesMonitor(Node):
    def __init__(self):
        super().__init__('steering_angles_monitor')

        # Subscribe to /joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)  # Use a small queue size (messages won't pile up)

        # Publisher for left & right steering angles
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/steering_angles',
            10)  # Small queue size is fine

        # Store latest steering angles
        self.left_steering_angle = 0.0
        self.right_steering_angle = 0.0

        # Publish at 500 Hz (2 ms interval)
        self.timer = self.create_timer(0.002, self.publish_steering_angles)

    def joint_state_callback(self, msg: JointState):
        # Ensure sufficient positions exist
        if len(msg.position) < 6:
            return  # Avoid index errors if message structure changes

        # Store the latest values
        self.left_steering_angle = msg.position[3]
        self.right_steering_angle = msg.position[5]

    def publish_steering_angles(self):
        # Publish the latest steering angles at a fixed 500 Hz rate
        steering_angles = Float32MultiArray()
        steering_angles.data = [self.left_steering_angle, self.right_steering_angle]
        self.publisher.publish(steering_angles)

def main(args=None):
    rclpy.init(args=args)
    node = SteeringAnglesMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

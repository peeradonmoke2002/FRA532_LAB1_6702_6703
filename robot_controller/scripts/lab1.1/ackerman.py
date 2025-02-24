#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

class AckermannController(Node):
    def __init__(self):
        super().__init__('ackermann_controller')
        
        # Declare and get parameters.
        self.declare_parameter("wheel_base", 0.2)
        self.declare_parameter("max_steering_angle", 0.5236)  # 30 degrees in radians
        self.wheel_base = self.get_parameter("wheel_base").value
        self.max_steering_angle = self.get_parameter("max_steering_angle").value

        # Subscriber: Listen to /cmd_vel (geometry_msgs/Twist)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publisher: Publish AckermannDriveStamped messages.
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            '/ackermann_cmd',
            10
        )
        
        self.get_logger().info("Ackermann Controller Node has been started.")

    def cmd_vel_callback(self, msg: Twist):
        # Extract linear and angular velocities.
        v = msg.linear.x   # Forward speed.
        w = msg.angular.z  # Yaw rate.

        # Compute the steering angle using the bicycle model:
        # If v is nearly zero, then set steering to zero.
        if np.abs(v) < 1e-6:
            steering_angle = 0.0
        else:
            # Formula: steering = arctan( (wheel_base * w) / v )
            steering_angle = np.arctan(self.wheel_base * w / v)
            # Limit the steering angle.
            steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)

        # Build and publish the Ackermann command.
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = self.get_clock().now().to_msg()
        ack_msg.header.frame_id = "base_link"
        ack_msg.drive.speed = v
        ack_msg.drive.steering_angle = steering_angle

        self.publisher.publish(ack_msg)
        self.get_logger().info(f"Published: speed = {v:.2f}, steering_angle = {steering_angle:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = AckermannController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

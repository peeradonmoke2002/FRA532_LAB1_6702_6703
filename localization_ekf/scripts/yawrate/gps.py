#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
import numpy as np

class GPSEmulator(Node):
    def __init__(self):
        super().__init__('gps_emulator')
        # Subscribe to ground truth data from Gazebo
        self.subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )
        # Publisher for the noisy odometry data
        self.publisher = self.create_publisher(Odometry, '/odom_noisy', 10)
        # Define GPS noise covariance for x and y positions.
        # Here, np.diag([0.05, 0.05]) creates a diagonal matrix with 0.05 on the diagonal.
        # Squaring it (i.e. [0.05^2, 0.05^2]) gives the variance for x and y.
        self.gps_noise_cov = np.diag([.05, 0.05]) ** 2

    def model_states_callback(self, msg: ModelStates):
        # Select the appropriate model by name, for example 'limo'
        try:
            idx = msg.name.index('limo')
        except ValueError:
            idx = 0

        true_pose = msg.pose[idx]
        true_twist = msg.twist[idx]

        # Generate noise for x and y using the covariance matrix
        noise_xy = np.linalg.cholesky(self.gps_noise_cov) @ np.random.randn(2, 1)


        # Create a noisy pose by adding the generated noise to the ground truth
        noisy_pose = Pose()
        noisy_pose.position.x = true_pose.position.x + float(noise_xy[0, 0])
        noisy_pose.position.y = true_pose.position.y + float(noise_xy[1, 0])

        noisy_pose.position.z = true_pose.position.z  # keep z unchanged (or add noise if needed)
        # Optionally, you can also add noise to orientation if desired.
        noisy_pose.orientation = true_pose.orientation

        # Here, we simply copy the twist data without noise.
        # You can add similar noise generation if you wish to simulate sensor noise in twist data.
        noisy_twist = Twist()
        noisy_twist.linear = true_twist.linear
        noisy_twist.angular = true_twist.angular

        # Create and populate an Odometry message with the noisy data
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose = noisy_pose
        odom_msg.twist.twist = noisy_twist

        # Publish the noisy odometry message
        self.publisher.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    gps_emulator_node = GPSEmulator()
    rclpy.spin(gps_emulator_node)
    gps_emulator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

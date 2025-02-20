#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (Point, Pose, Quaternion, Twist, Vector3, TransformStamped)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
import tf_transformations
import math
import numpy as np

class Odo1Track(Node):
    def __init__(self):
        super().__init__('odo1track')
        queue_size = 10
        self.dt_loop = 1 / 100.0  # 100 Hz update rate

        # ✅ Robot parameters
        self.wheel_base = 0.2         # Meters
        self.wheel_radius = 0.045     # Meters
        self.max_steering_angle = 0.523598767  # 30 degrees in radians

        # ✅ Publishers and timer
        self.publisher = self.create_publisher(Odometry, '/odom', queue_size)
        self.publisher_imu = self.create_publisher(Float32, '/imu_degree', queue_size)
        self.timer = self.create_timer(self.dt_loop, self.timer_callback)

        # ✅ Subscribers
        self.subscription_jointstate = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, queue_size)
        self.subscription_imu = self.create_subscription(
            Imu, '/imu', self.feedback_imu, queue_size)

        # ✅ Transform broadcaster
        self.tf_br = TransformBroadcaster(self)

        # ✅ Initial state
        self.robot_position = np.array([9.073500, 0.0, 1.57])  # x, y, yaw
        self.orientation = tf_transformations.quaternion_from_euler(0.0, 0.0, self.robot_position[2])

        # ✅ State variables
        self.relative_yaw = 0.0
        self.wheel_speed = 0.0
        self.steering_angle = 0.0  # Single-track model uses ONE steering angle
        self.last_callback_time = self.get_clock().now()
        self.initial_orientation = None

    def joint_state_callback(self, msg):
        """ Reads wheel speed and steering angle from joint states """
        # ✅ Extract rear wheel velocity (Assuming index 2)
        self.wheel_speed = msg.velocity[2] * self.wheel_radius  # Convert to linear speed

        # ✅ Extract front steering angle (Assuming index 3 for front wheel)
        self.steering_angle = msg.position[3]  # Single front wheel steering

    def feedback_imu(self, msg):
        """ Reads IMU orientation and extracts yaw """
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        if orientation_list == [0.0, 0.0, 0.0, 0.0]:
            return  # Skip invalid IMU data
        
        _, _, yaw = tf_transformations.euler_from_quaternion(orientation_list)
        if self.initial_orientation is None:
            self.initial_orientation = yaw  # Store initial IMU orientation
        self.relative_yaw = yaw - self.initial_orientation  # Compute yaw change

    def compute_kinematic_single_track(self, v, beta_F):
        """ Compute rotation rate (omega) using the single-track model """
        if abs(beta_F) < 1e-6:  # Going straight
            return 0.0
        
        # ✅ Compute omega using the derived equation
        omega = (v / self.wheel_base) * (math.tan(beta_F) / math.cos(0.0))  # beta_R = 0
        return omega

    def timer_callback(self):
        """ Compute odometry using the Kinematic-Single-Track Model """
        v = self.wheel_speed  # Rear wheel velocity
        beta_F = self.steering_angle  # Front steering angle

        # ✅ Compute yaw rate using kinematic model
        yaw_rate = self.compute_kinematic_single_track(v, beta_F)

        # ✅ Compute new pose using kinematic single-track equations
        absolute_yaw = self.robot_position[2] + yaw_rate * self.dt_loop
        vx = v * math.cos(absolute_yaw)
        vy = v * math.sin(absolute_yaw)

        self.robot_position[0] += vx * self.dt_loop  # Update X
        self.robot_position[1] += vy * self.dt_loop  # Update Y
        self.robot_position[2] = absolute_yaw  # Update Yaw

        # ✅ Convert yaw to quaternion
        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, absolute_yaw)

        # ✅ Create and publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose = Pose(
            position=Point(x=self.robot_position[0], y=self.robot_position[1], z=0.0),
            orientation=Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
        )

        odom_msg.twist.twist.linear = Vector3(x=vx, y=vy, z=0.0)
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=yaw_rate)

        self.publisher.publish(odom_msg)
        self.publisher_imu.publish(Float32(data=self.relative_yaw * 180 / math.pi))

        # ✅ Publish the transform for tf2
        transform = TransformStamped()
        transform.header.stamp = odom_msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = odom_msg.pose.pose.position.x
        transform.transform.translation.y = odom_msg.pose.pose.position.y
        transform.transform.translation.z = odom_msg.pose.pose.position.z
        transform.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_br.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = Odo1Track()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

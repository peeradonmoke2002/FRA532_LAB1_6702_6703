#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (Point, Pose, PoseWithCovariance, Quaternion, 
                               Twist, TwistWithCovariance, Vector3, TransformStamped)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Header, Float32
from tf2_ros import TransformBroadcaster
import tf_transformations
import math
import numpy as np

class YawrateOdom(Node):
    def __init__(self):
        super().__init__('YawrateOdom')
        queue_size = 10
        self.dt_loop = 1 / 100.0  # 100 Hz update rate

        # Robot parameters
        self.wheel_base = 0.2         # meters
        self.track_width = 0.14       # meters
        self.wheel_radius = 0.045     # meters

        # Publishers and timer
        self.publisher = self.create_publisher(Odometry, '/odom', queue_size)
        self.publisher_imu = self.create_publisher(Float32, '/imu_degree', queue_size)
        self.timer = self.create_timer(self.dt_loop, self.timer_callback)

        # Subscribers
        self.subscription_joinstate = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, queue_size)
        self.subscription_imu = self.create_subscription(
            Imu, '/imu', self.feedback_imu, queue_size)

        # Transform broadcaster
        self.tf_br = TransformBroadcaster(self)

        # Covariance matrices (optional – currently commented out in messages)
        self.pose_cov = np.diag([1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9])
        self.twist_cov = np.diag([1.0e-9, 1.0e-6, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9])
        
        # Spawn pose: Adjust these values so they match your simulation’s initial pose if needed.
        spawn_x_val = 9.073500
        spawn_y_val = 0.0
        spawn_yaw_val = 1.57  # In radians; note that this should be consistent with your IMU's frame.
        self.robot_position = np.array([spawn_x_val, spawn_y_val, spawn_yaw_val])
        self.orientation = tf_transformations.quaternion_from_euler(0.0, 0.0, self.robot_position[2])

        # Odometry state variables
        self.relative_yaw = 0.0  # Change in yaw from the initial IMU reading
        self.absolute_yaw = 0.0  # Absolute yaw for integrating the robot’s position
        self.wheelspeed = 0.0
        self.wheel_omega = np.array([0.0, 0.0])
        self.last_callback_time = self.get_clock().now()
        self.initial_orientation = None  # To store the initial IMU yaw

    def joint_state_callback(self, msg):
        # Extract rear wheel velocities (assumed indices 2 and 4)
        self.wheel_omega = np.array([msg.velocity[4], msg.velocity[2]])
        # Extract steering angles (assumed indices 3 and 5, if needed later)
        self.steering_angles = np.array([msg.position[3], msg.position[5]])

    def get_wheel_speed(self, wheel_omega: np.ndarray) -> float:
        # Convert average wheel angular speed to linear speed
        wheelspeed = ((wheel_omega[0] + wheel_omega[1]) / 2.0) * self.wheel_radius
        return wheelspeed
    

    def feedback_imu(self, msg):
        # Extract orientation from the IMU and convert to Euler yaw
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        if orientation_list == [0.0, 0.0, 0.0, 0.0]:
            return  # Skip invalid IMU data
        
        _, _, yaw = tf_transformations.euler_from_quaternion(orientation_list)
        if self.initial_orientation is None:
            self.initial_orientation = yaw
        self.relative_yaw = yaw - self.initial_orientation

    def timer_callback(self):
        self.wheelspeed = self.get_wheel_speed(self.wheel_omega)
        self.absolute_yaw = self.robot_position[2] + self.relative_yaw
        vx = self.wheelspeed * math.cos(self.absolute_yaw)
        vy = self.wheelspeed * math.sin(self.absolute_yaw)
        # Update the robot’s position
        self.robot_position[0] += vx * self.dt_loop
        self.robot_position[1] += vy * self.dt_loop

        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, self.absolute_yaw)

        # Create and populate the Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose = Pose(
            position=Point(
                x=self.robot_position[0],
                y=self.robot_position[1],
                z=0.0  # Remains 0 for a planar robot
            ),
            orientation=Quaternion(
                x=quaternion[0],
                y=quaternion[1],
                z=quaternion[2],
                w=quaternion[3]
            )
        )
        odom_msg.pose.covariance = self.pose_cov.flatten()

        odom_msg.twist.twist.linear = Vector3(x=vx, y=0.0, z=0.0)
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=self.absolute_yaw)
        odom_msg.twist.covariance = self.twist_cov.flatten()

        self.publisher.publish(odom_msg)
        self.publisher_imu.publish(Float32(data=self.relative_yaw * 180 / math.pi))

        # Publish the transform for tf2
        transform = TransformStamped()
        transform.header.stamp = odom_msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = odom_msg.pose.pose.position.x
        transform.transform.translation.y = odom_msg.pose.pose.position.y
        transform.transform.translation.z = odom_msg.pose.pose.position.z
        transform.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_br.sendTransform(transform)

        print('x: ', self.robot_position[0], 'y: ', self.robot_position[1], 'z', quaternion[2], 'w', quaternion[3])

def main(args=None):
    rclpy.init(args=args)
    pub_odom_node = YawrateOdom()
    rclpy.spin(pub_odom_node)
    pub_odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

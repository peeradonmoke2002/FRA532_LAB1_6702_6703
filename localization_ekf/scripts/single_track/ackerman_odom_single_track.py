#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (Point, Pose, PoseWithCovariance, Quaternion, Twist, 
                               TwistWithCovariance, Vector3, TransformStamped)
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Float64MultiArray
from std_msgs.msg import Header, Float32

from tf2_ros import TransformBroadcaster
import tf_transformations
from sensor_msgs.msg import JointState
import tf2_ros
import numpy as np


class SingleTrackOdom(Node):
    def __init__(self):
        super().__init__('single_track_odom')
        # Robot Parameters
        self.wheel_base = 0.2  # Distance between front and rear axles (meters)
        self.track_width = 0.14  # Distance between left and right wheels (meters)
        self.wheel_radius = 0.045  # Rear wheel radius (meters)
        
        # Initialize Pose and Velocity
        self.x_curr = 0.0
        self.y_curr = 0.0
        self.theta_curr = 0.0
        self.v_curr = 0.0
        self.w_curr = 0.0  # Yaw rate
        self.quat = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta_curr)

        self.x_prev = 9.073500
        self.y_prev = 0.0
        self.theta_prev = 1.57  # In radians
        self.v_prev = 0.0
        self.w_prev = 0.0  # Yaw rate

        self.pose_cov = np.diag([1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9])
        self.twist_cov = np.diag([1.0e-9, 1.0e-6, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9])


        # TF broadcaster
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.publish_transform = TransformBroadcaster(self)

        # Joint States Subscriber (Wheel velocities and steering angles)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Timer for Odometry Updates
        self.dt_loop = 1 / 100  
        self.timer = self.create_timer(self.dt_loop, self.timer_callback)

        self.v_rl = 0.0
        self.v_rr = 0.0
        self.delta_fl = 0.0
        self.delta_fr = 0.0
        self.delta_avg = 0.0

        self.prev_time = self.get_clock().now()


    def joint_state_callback(self, msg):
        if ('rear_left_wheel' in msg.name and 'rear_right_wheel' in msg.name and
            'front_left_steering' in msg.name and 'front_right_steering' in msg.name):

            left_wheel_index = msg.name.index('rear_left_wheel')
            right_wheel_index = msg.name.index('rear_right_wheel')
            left_steering_index = msg.name.index('front_left_steering')
            right_steering_index = msg.name.index('front_left_steering')

            # Get rear wheel velocities (convert encoder rate to linear velocity)
            self.v_rl = msg.velocity[left_wheel_index] * self.wheel_radius
            self.v_rr = msg.velocity[right_wheel_index] * self.wheel_radius

            # Extract measured steering angles (if needed)
            self.delta_fl = msg.position[left_steering_index]
            self.delta_fr = msg.position[right_steering_index]



    def timer_callback(self):
        # Compute elapsed time dt (in seconds)
        dt = (self.get_clock().now() - self.prev_time).to_msg().nanosec * 1.0e-9

        self.x_curr = self.x_prev + self.v_prev * dt * np.cos(self.theta_prev + (self.w_prev * dt / 2))
        self.y_curr = self.y_prev + self.v_prev * dt * np.sin(self.theta_prev + (self.w_prev * dt / 2))
        self.theta_curr = self.theta_prev + self.w_prev * dt
        self.quat = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta_curr)

        self.v_curr = (self.v_rl + self.v_rr) / 2.0
        delta_avg = (self.delta_fl + self.delta_fr) / 2.0
        self.w_curr = (self.v_curr / self.wheel_base) * np.tan(delta_avg)

        # Compute twist (linear velocity in world frame)
        vx = self.v_curr * np.cos(self.theta_curr)
        vy = self.v_curr * np.sin(self.theta_curr)

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position = Point(x=self.x_curr, y=self.y_curr, z=0.0)
        odom_msg.pose.pose.orientation = Quaternion(
            x=self.quat[0],
            y=self.quat[1],
            z=self.quat[2],
            w=self.quat[3]
        )
        odom_msg.pose.covariance = self.pose_cov.flatten()
        odom_msg.twist.twist.linear = Vector3(x=self.v_curr, y=0.0, z=0.0)
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=self.w_curr)
        odom_msg.twist.covariance = self.twist_cov.flatten()
        self.odom_pub.publish(odom_msg)

        # Publish the TF transform
        transform = TransformStamped()
        transform.header.stamp = odom_msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = self.x_curr
        transform.transform.translation.y = self.y_curr
        transform.transform.rotation = Quaternion(
            x=self.quat[0],
            y=self.quat[1],
            z=self.quat[2],
            w=self.quat[3]
        )
        # uncomment for testing for all models !!
        # self.publish_transform.sendTransform(transform)

        # Update previous state for the next iteration
        self.prev_time = self.get_clock().now()
        self.x_prev = self.x_curr
        self.y_prev = self.y_curr
        self.v_prev = self.v_curr
        self.w_prev = self.w_curr
        self.theta_prev = self.theta_curr

        # print('x:', self.x_curr, 'y:', self.y_curr, 'yaw rate:', self.w_curr)






def main(args=None):
    rclpy.init(args=args)
    node = SingleTrackOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
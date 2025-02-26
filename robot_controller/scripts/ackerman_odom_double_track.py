#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (Point, Pose, PoseWithCovariance, Quaternion, Twist, 
                               TwistWithCovariance, Vector3, TransformStamped)
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import tf_transformations
from sensor_msgs.msg import JointState
import tf2_ros
import numpy as np


class DoubleTrackOdom(Node):
    def __init__(self):
        super().__init__('double_track_odom')
        self.max_steering_angle = np.deg2rad(30)  
        self.wheel_base = 0.2 
        self.track_width = 0.14 
        self.wheel_radius = 0.045 
        self.r_rl = [0.0, 0.07]  
        self.r_rr = [0.0, -0.07]  

        # Pose and velocity state (set initial conditions as needed)
        self.x_curr = 0.0
        self.y_curr = 0.0
        self.theta_curr = 0.0
        self.v_curr = 0.0
        self.w_curr = 0.0           
        self.quat = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta_curr)

        self.x_prev = 9.073500
        self.y_prev = 0.0
        self.theta_prev = 1.57     
        self.v_prev = 0.0
        self.w_prev = 0.0

        # TF broadcaster
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.publish_transform = TransformBroadcaster(self)

        # Subscriptions and publisher
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'double_track/odom', 10)

        self.dt_loop = 1 / 100  # 100 Hz update rate
        self.timer = self.create_timer(self.dt_loop, self.timer_callback)
        self.prev_time = self.get_clock().now()

        self.pose_cov = np.diag([1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9])
        self.twist_cov = np.diag([1.0e-9, 1.0e-6, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9])

        self.v_rl = 0.0
        self.v_rr = 0.0
        self.delta_fl = 0.0
        self.delta_fr = 0.0


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


    # Equation (5) for computing vehicle velocity (v)
    def compute_vehicle_velocity(self, v_rl, v_rr, delta_fl, delta_fr, rl_x, rl_y, rr_x, rr_y):
        """Compute vehicle velocity components (v) using Equation (5)."""
        N1 = rl_x * v_rr * np.sin(delta_fl)
        N2 = rl_y * v_rr * np.cos(delta_fl)
        N3 = rr_x * v_rl * np.sin(delta_fr)
        N4 = rr_y * v_rl * np.cos(delta_fr)

        D1 = rl_x * np.sin(delta_fl) * np.cos(delta_fr)
        D2 = rl_y * np.cos(delta_fl) * np.cos(delta_fr)
        D3 = rr_x * np.sin(delta_fr) * np.cos(delta_fl)
        D4 = rr_y * np.cos(delta_fr) * np.cos(delta_fl)

        v = (N1 - N2 - N3 + N4) / (D1 - D2 - D3 + D4)
        return v

    # Equation (6) for computing yaw rate (ω)
    def compute_yaw_rate(self, v_rl, v_rr, delta_fl, delta_fr, rl_x, rl_y, rr_x, rr_y):
        """Compute yaw rate (ω) using Equation (6)."""
        N1 = v_rl * np.cos(delta_fr)
        N2 = v_rr * np.cos(delta_fl)

        D1 = rl_x * np.sin(delta_fl) * np.cos(delta_fr)
        D2 = rl_y * np.cos(delta_fl) * np.cos(delta_fr)
        D3 = rr_x * np.sin(delta_fr) * np.cos(delta_fl)
        D4 = rr_y * np.cos(delta_fr) * np.cos(delta_fl)

        w = (N1 - N2) / (D1 - D2 - D3 + D4)
        
        return w
    

    

    def timer_callback(self):
        # Compute time step dt
        dt = (self.get_clock().now() - self.prev_time).to_msg().nanosec * 1.0e-9

        self.x_curr = self.x_prev + self.v_prev * dt * np.cos(self.theta_prev + (self.w_prev * dt / 2))
        self.y_curr = self.y_prev + self.v_prev * dt * np.sin(self.theta_prev + (self.w_prev * dt / 2))
        self.theta_curr = self.theta_prev + self.w_prev * dt
        self.quat = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta_curr)

        self.v_curr = (self.v_rl + self.v_rr) / 2
        self.w_curr = (self.v_rr - self.v_rl) / (self.r_rr[1]-self.r_rl[1])

        # for testing concept !! ##
        # self.v_curr = self.compute_vehicle_velocity(self.v_rl, self.v_rr,
        #                                     self.delta_fl, self.delta_fr,
        #                                     self.r_rl[0], self.r_rl[1],
        #                                     self.r_rr[0], self.r_rr[1])
        # self.w_curr = self.compute_yaw_rate(self.v_rl, self.v_rr,
        #                             self.delta_fl, self.delta_fr,
        #                             self.r_rl[0], self.r_rl[1],
        #                             self.r_rr[0], self.r_rr[1])

        avg_steering = (self.delta_fl + self.delta_fr) / 2
        real_sign = np.sign(avg_steering)
        if np.sign(self.w_curr) != real_sign:
            self.w_curr = -self.w_curr 

        


        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position = Point(x=self.x_curr, y=self.y_curr, z=0.0)
        odom_msg.pose.pose.orientation = Quaternion(x=self.quat[0],
                                                    y=self.quat[1],                
                                                    z=self.quat[2],
                                                    w=self.quat[3])
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
        transform.transform.rotation = Quaternion(x=self.quat[0], y=self.quat[1], z=self.quat[2], w=self.quat[3])
        
        # uncomment for testing for all models !!
        # self.publish_transform.sendTransform(transform)

        # Update previous state for next iteration
        self.prev_time = self.get_clock().now()
        self.x_prev = self.x_curr
        self.y_prev = self.y_curr
        self.v_prev = self.v_curr
        self.w_prev = self.w_curr
        self.theta_prev = self.theta_curr

        # print('x:', self.x_curr, 'y:', self.y_curr, 'yaw rate:', self.w_curr)
        # print("speed left: ", self.v_rl, "speed right: ", self.v_rr, "yaw rate: ", self.w_curr)




def main(args=None):
    rclpy.init(args=args)
    node = DoubleTrackOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
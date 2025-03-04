#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
import math

############################################
# Minimal EKF Implementation (3-State)
############################################

############################################
# ROS 2 Node for Minimal EKF Fusion
############################################

class MinimalEKFNode(Node):
    def __init__(self):
        super().__init__('minimal_ekf_node')
        self.ekf = MinimalEKF()
        
        # Publisher for estimated state as Odometry message
        self.pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        
        # Subscribers:
        # Odometry subscriber (for dead reckoning)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # GPS subscriber using PoseStamped messages
        self.create_subscription(PoseStamped, '/gps', self.gps_callback, 10)
        
        # Timer for running the prediction step
        self.timer = self.create_timer(self.ekf.dt, self.timer_callback)
        
        # For demonstration, use a constant control input: [v, omega]
        self.control_input = [1.0, 0.1]
    
    def timer_callback(self):
        # Run the prediction step
        self.ekf.predict(self.control_input)
        self.publish_state()

    def odom_callback(self, msg):
        # Extract odometry measurement: [x, y, theta]
        z = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        ])
        y, K, S = self.ekf.update_odom(z)
        self.get_logger().info(f"Odom update - Residual: {y}, Updated state: {self.ekf.x_est}")
    
    def gps_callback(self, msg):
        # Extract GPS measurement from PoseStamped message: only [x, y]
        z = np.array([
            msg.pose.position.x,
            msg.pose.position.y
        ])
        y, K, S = self.ekf.update_gps(z)
        self.get_logger().info(f"GPS update - Residual: {y}, Updated state: {self.ekf.x_est}")
    
    def get_yaw_from_quaternion(self, quat):
        euler = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return euler[2]
    
    def publish_state(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.pose.position.x = float(self.ekf.x_est[0])
        msg.pose.pose.position.y = float(self.ekf.x_est[1])
        msg.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, self.ekf.x_est[2])
        msg.pose.pose.orientation = Quaternion(
            x=q[0], y=q[1], z=q[2], w=q[3]
        )
        self.pub.publish(msg)
        self.get_logger().debug("Published EKF estimated state.")


class MinimalEKF:
    def __init__(self):
        



        # PID basic
        # State vector: [x, y, theta]
        self.x_est = np.array([0.0, 0.0, 0.0015])  # initial estimate
        self.P_est = np.eye(3) * 1.0            # initial covariance matrix

        # Process noise covariance matrix (3x3)
        # Lower values indicate higher confidence in the motion model.
        self.Q = np.diag([0.0155, 0.0155, 0.0155])
        
        # Measurement noise covariance matrix for GPS (2x2)
        self.R = np.diag([0.02, 0.02])
        
        # Measurement noise covariance for odometry (3x3)
        self.R_odom = np.diag([0.048, 0.048, 0.048])
        
        # Time step (s)
        self.dt = 0.01

        # purepursuit and stanley basic

        self.x_est = np.array([0.0, 0.0, 0.001])  # initial estimate
        self.P_est = np.eye(3) * 1.0            # initial covariance matrix

        # Process noise covariance matrix (3x3)
        # Lower values indicate higher confidence in the motion model.
        self.Q = np.diag([0.010, 0.010, 0.010])
        
        # Measurement noise covariance matrix for GPS (2x2)
        self.R = np.diag([0.02, 0.02])
        
        # Measurement noise covariance for odometry (3x3)
        self.R_odom = np.diag([0.05, 0.05, 0.05])
        
        # Time step (s)
        self.dt = 0.01

    def motion_model(self, x, u):
        """
        Motion model with control input u = [v, omega]:
          x_{k+1} = x_k + v*cos(theta)*dt
          y_{k+1} = y_k + v*sin(theta)*dt
          theta_{k+1} = theta_k + omega*dt
        """
        v, omega = u
        theta = x[2]
        x_next = np.zeros(3)
        x_next[0] = x[0] + v * math.cos(theta) * self.dt
        x_next[1] = x[1] + v * math.sin(theta) * self.dt
        x_next[2] = x[2] + omega * self.dt
        return x_next

    def jacobian_F(self, x, u):
        """
        Jacobian of the motion model with respect to the state.
        """
        v, _ = u
        theta = x[2]
        F = np.eye(3)
        F[0, 2] = -v * math.sin(theta) * self.dt
        F[1, 2] =  v * math.cos(theta) * self.dt
        return F

    def jacobian_H(self, x):
        """
        Measurement model: z = [x, y]^T.
        Hence, H is:
            [1 0 0]
            [0 1 0]
        """
        H = np.array([
            [1, 0, 0],
            [0, 1, 0]
        ])
        return H

    def predict(self, u):
        """
        Prediction step: propagate state and predict state covariance.
        """
        F = self.jacobian_F(self.x_est, u)
        self.x_est = self.motion_model(self.x_est, u)
        self.P_est = F @ self.P_est @ F.T + self.Q

    def update_odom(self, z):
        """
        Update step with odometry measurement.
        z is expected to be [x, y, theta]^T.
        """
        H = np.eye(3)
        z_pred = H @ self.x_est
        y = z - z_pred  # innovation
        S = H @ self.P_est @ H.T + self.R_odom
        K = self.P_est @ H.T @ np.linalg.inv(S)
        self.x_est = self.x_est + K @ y
        self.P_est = (np.eye(3) - K @ H) @ self.P_est
        return y, K, S

    def update_gps(self, z):
        """
        Update step with GPS measurement.
        z is expected to be [x, y]^T.
        """
        H = np.array([[1, 0, 0],
                      [0, 1, 0]])
        z_pred = H @ self.x_est
        y = z - z_pred  # innovation
        S = H @ self.P_est @ H.T + self.R
        K = self.P_est @ H.T @ np.linalg.inv(S)
        self.x_est = self.x_est + K @ y
        self.P_est = (np.eye(3) - K @ H) @ self.P_est
        return y, K, S


def main(args=None):
    rclpy.init(args=args)
    node = MinimalEKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

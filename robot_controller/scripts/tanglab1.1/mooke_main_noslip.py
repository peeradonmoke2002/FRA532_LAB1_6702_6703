#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class AckermannController(Node):
    def __init__(self):
        super().__init__('ackermann_controller')
        
        # Robot parameters
        self.wheel_base = 0.2         # Distance between front and rear axles (meters)
        self.track_width = 0.14      # Distance between left and right wheels (meters)
        self.max_steering_angle = 0.523598767  # 30 degrees in radians
        self.wheel_radius = 0.04815      # Rear wheel radius (meters)

        # Initialize variables
        self.cmd_vel = [0.0, 0.0]
        self.wheel_speed = 0.0
        self.steer_angles = 0.0
        self.ackermann_percentage = 0.0
        
        # Create publisher for steering commands
        self.pub_steering = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_position_controller/joint_trajectory',
            10
        )
        # Publisher for drive (wheel speed) commands
        self.pub_wheel_spd = self.create_publisher(
            Float64MultiArray, 
            '/velocity_controllers/commands', 
            10
        )
        
        # Subscribe to cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_timer(0.001, self.timer_callback)

    def cmd_vel_callback(self, msg):
        self.cmd_vel = [msg.linear.x, msg.angular.z]

    
    def publish_steering(self, left_angle, right_angle):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['front_left_steering', 'front_right_steering']

        point = JointTrajectoryPoint()
        point.positions = [float(left_angle), float(right_angle)]
        point.time_from_start = rclpy.duration.Duration(seconds=0.1).to_msg()
        
        traj_msg.points.append(point)
        self.pub_steering.publish(traj_msg)


    def cot(self, x):
        return np.cos(x) / np.sin(x)
    
    def compute_ackermann_angles(self, center_steer: float):
        if np.abs(center_steer) < 1e-6:
            return 0.0, 0.0

        tan_steer = np.tan(center_steer)
        # Corrected: inner (inside) wheel uses L - 0.5 * track_width, outer uses L + 0.5 * track_width.
        left_angle = np.arctan((self.wheel_base * tan_steer) / 
                                (self.wheel_base + 0.5 * self.track_width * tan_steer))
        right_angle = np.arctan((self.wheel_base * tan_steer) / 
                                (self.wheel_base - 0.5 * self.track_width * tan_steer))
        # left_angle = np.clip(left_angle, -self.max_steering_angle, self.max_steering_angle)
        # right_angle = np.clip(right_angle, -self.max_steering_angle, self.max_steering_angle)
        return left_angle, right_angle


    def timer_callback(self):
        # Compute wheel speed (assume wheel speed = forward speed / wheel radius)
        v_x = self.cmd_vel[0]  # forward linear speed
        wheel_speed_left = v_x / self.wheel_radius
        wheel_speed_right = v_x / self.wheel_radius

        # Check for straight or nearly straight motion:
        if abs(self.cmd_vel[0]) < 1e-9 or abs(self.cmd_vel[1]) < 1e-9:
            self.steer_angle_center = 0.0
            left_angle, right_angle = 0.0, 0.0
            self.publish_steering(left_angle, right_angle)
 
        else:
            # base from tan(max_steering_angle) = L/R
            r_ICR = self.wheel_base / np.tan(self.max_steering_angle) 
            turn_sign = np.sign(self.cmd_vel[1])  # +1 for left turn, -1 for right turn
            self.steer_angle_center = np.arctan(self.wheel_base / r_ICR) * turn_sign
            inside_angle, outside_angle = self.compute_ackermann_angles(self.steer_angle_center)
            left_angle = outside_angle    
            right_angle = inside_angle  

            if turn_sign > 0:
                wheel_speed_left = self.cmd_vel[0] * (2 - (self.track_width / r_ICR)) / (2 * self.wheel_radius)
                wheel_speed_right = self.cmd_vel[0] * (2 + (self.track_width / r_ICR)) / (2 * self.wheel_radius)
            else:
                wheel_speed_left = self.cmd_vel[0] * (2 + (self.track_width / r_ICR)) / (2 * self.wheel_radius)
                wheel_speed_right = self.cmd_vel[0] * (2 - (self.track_width / r_ICR)) / (2 * self.wheel_radius)
        
            self.publish_steering(left_angle, right_angle)

        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed_left, wheel_speed_right]
        self.pub_wheel_spd.publish(wheel_speed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

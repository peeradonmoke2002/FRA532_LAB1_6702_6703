#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rclpy.duration import Duration
import numpy as np

class BicycleController(Node):
    def __init__(self):
        super().__init__('bicycle_controller')
        self.dt_loop = 1/100.0  # Loop time in seconds
        
        # Robot parameters (bicycle model)
        self.wheel_base = 0.2         # L: Distance between front and rear axles (meters)
        self.wheel_radius = 0.045     # r: Rear wheel radius (meters)
        self.max_steering_angle = np.deg2rad(30)  # Maximum steering angle (30 degrees in radians)
        
        # Command variables
        self.cmd_vel = [0.0, 0.0]     # [linear velocity (v), angular velocity (w)] from /cmd_vel
        self.steering_angle = 0.0     # Center steering angle (delta)
        self.wheel_speed = 0.0        # Rear wheel angular speed
        
        # Publishers
        self.pub_steering = self.create_publisher(
            Float64MultiArray,
            "/position_controllers/commands",
            10
        )
        self.pub_wheel_spd = self.create_publisher(
            Float64MultiArray, 
            '/velocity_controllers/commands', 
            10
        )
        
        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Timer
        self.create_timer(self.dt_loop, self.timer_callback)

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel = [msg.linear.x, msg.angular.z]

    def publish_steering(self, steering_L: float, steering_R: float):
        steering_msg = Float64MultiArray()
        steering_msg.data = [steering_L, steering_R]
        self.pub_steering.publish(steering_msg)

    def publish_wheel_speed(self, wheelspeed_L:float, wheelspeed_R:float):
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [wheelspeed_L, wheelspeed_R]
        self.pub_wheel_spd.publish(wheel_msg)

    def timer_callback(self):
        v = self.cmd_vel[0]
        w = self.cmd_vel[1]
        
        if np.abs(v) < 1e-9:
            self.steering_angle = 0.0
            self.wheel_speed = 0.0
        else:
            # From w = v/L * tan(delta)  =>  delta = arctan(L*w / v)
            self.steering_angle = np.arctan(self.wheel_base * w / v)
            self.wheel_speed = v / self.wheel_radius
        
        # Publish the steering command
        self.publish_steering(self.steering_angle, self.steering_angle)

        # Publish wheel speeds
        self.publish_wheel_speed(self.wheel_speed, self.wheel_speed)
        # print(f"v = {v:.3f}, w = {w:.3f}, delta = {self.steering_angle:.3f}, wheel_speed = {self.wheel_speed:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = BicycleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

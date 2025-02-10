#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class BicycleController(Node):
    def __init__(self):
        super().__init__('bicycle_controller')
        self.dt_loop = 1/50.0  # Loop time in seconds
        
        # Robot parameters (bicycle model)
        self.wheel_base = 0.2         # L: Distance between front and rear axles (meters)
        self.wheel_radius = 0.045     # r: Rear wheel radius (meters)
        self.max_steering_angle = 0.523598767  # Maximum steering angle (30 degrees in radians)
        
        # Command variables
        self.cmd_vel = [0.0, 0.0]     # [linear velocity (v), angular velocity (w)] from /cmd_vel
        self.steering_angle = 0.0     # Center steering angle (delta)
        self.wheel_speed = 0.0        # Rear wheel angular speed
        
        # Publishers
        self.pub_steering = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_position_controller/joint_trajectory',
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

    def publish_steering(self, steering: float):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['front_left_steering', 'front_right_steering']
        
        point = JointTrajectoryPoint()
        point.positions = [float(steering), float(steering)]
        # Set a small time-from-start, e.g., 0.1 seconds.
        from rclpy.duration import Duration
        point.time_from_start = Duration(seconds=0.1).to_msg()
        traj_msg.points.append(point)
        self.pub_steering.publish(traj_msg)

    def timer_callback(self):
        # Extract commands: v is forward speed, w is yaw rate.
        v = self.cmd_vel[0]
        w = self.cmd_vel[1]
        
        if np.abs(v) < 1e-9:
            self.steering_angle = 0.0
            self.wheel_speed = 0.0
        else:
            # From w = v/L * tan(delta)  =>  delta = arctan(L*w / v)
            self.steering_angle = np.arctan(self.wheel_base * w / v)
            self.steering_angle = np.clip(self.steering_angle, -self.max_steering_angle, self.max_steering_angle)
            self.wheel_speed = v / self.wheel_radius
        
        # Publish the steering command.
        self.publish_steering(self.steering_angle)
        
        # Publish the wheel speed command.
        wheel_msg = Float64MultiArray()
        # Assuming a differential drive controller where both wheels are commanded identically.
        wheel_msg.data = [self.wheel_speed, self.wheel_speed]
        self.pub_wheel_spd.publish(wheel_msg)
        
        # (Optional) Log info for debugging.
        # self.get_logger().info(f"v = {v:.3f}, w = {w:.3f}, delta = {self.steering_angle:.3f}, wheel_speed = {self.wheel_speed:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = BicycleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

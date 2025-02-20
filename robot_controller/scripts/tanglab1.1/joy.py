#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rclpy.duration import Duration
import numpy as np

class BicycleController(Node):
    def __init__(self):
        super().__init__('bicycle_controller')
        self.dt_loop = 1 / 50.0  # Loop time in seconds (50 Hz)

        # 🚗 Robot parameters (bicycle model)
        self.wheel_base = 0.2         # L: Distance between front and rear axles (meters)
        self.wheel_radius = 0.045     # r: Rear wheel radius (meters)
        self.max_steering_angle = 0.523598767  # Maximum steering angle (30 degrees in radians)

        # 🎮 Joystick Mapping
        self.joy_axes = [0.0] * 8     # Placeholder for joystick axes
        self.joy_buttons = [0] * 12   # Placeholder for joystick buttons
        self.deadzone = 0.1           # Deadzone for small joystick movements
        self.max_linear_speed = 1.0   # Max forward speed (m/s)
        self.max_angular_speed = 1.5  # Max turning rate (rad/s)

        # 🚀 Publishers
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

        # 🎮 Subscribers
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # ⏳ Timer
        self.create_timer(self.dt_loop, self.timer_callback)

        self.get_logger().info("✅ Bicycle Controller with Joystick Support Started!")

    def joy_callback(self, msg: Joy):
        """ Callback function to process joystick input. """
        self.joy_axes = msg.axes
        self.joy_buttons = msg.buttons

    def get_velocity_from_joy(self):
        """ Convert joystick input to velocity (linear v, angular w). """
        v = self.joy_axes[1] * self.max_linear_speed  # Left Stick Up/Down for forward/backward
        w = self.joy_axes[3] * self.max_angular_speed  # Right Stick Left/Right for turning

        # Apply deadzone
        if abs(v) < self.deadzone:
            v = 0.0
        if abs(w) < self.deadzone:
            w = 0.0

        return v, w

    def publish_steering(self, steering: float):
        """ Publish steering angle to the front wheel actuators. """
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['front_left_steering', 'front_right_steering']
        
        point = JointTrajectoryPoint()
        point.positions = [float(steering), float(steering)]
        point.time_from_start = Duration(seconds=0.1).to_msg()
        
        traj_msg.points.append(point)
        self.pub_steering.publish(traj_msg)


    def smooth_velocity(self, v_new, w_new, alpha=0.2):
        """ใช้ EMA smoothing เพื่อลดอาการกระตุก"""
        self.v = alpha * v_new + (1 - alpha) * self.v
        self.w = alpha * w_new + (1 - alpha) * self.w

    def timer_callback(self):
        """ Timer loop to update and publish control commands. """
        # Extract velocity commands from joystick
        v, w = self.get_velocity_from_joy()

        if np.abs(v) < 1e-9:
            self.steering_angle = 0.0
            self.wheel_speed = 0.0
        else:
            # Compute steering angle using bicycle kinematics
            self.steering_angle = np.arctan(self.wheel_base * w / v)
            self.steering_angle = np.clip(self.steering_angle, -self.max_steering_angle, self.max_steering_angle)
            self.wheel_speed = v / self.wheel_radius

        # Publish the steering command
        self.publish_steering(self.steering_angle)

        # Publish the wheel speed command
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [self.wheel_speed, self.wheel_speed]
        self.pub_wheel_spd.publish(wheel_msg)

        # (Optional) Debugging log
        self.get_logger().info(f"🎮 v: {v:.3f}, w: {w:.3f}, delta: {self.steering_angle:.3f}, wheel_speed: {self.wheel_speed:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = BicycleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

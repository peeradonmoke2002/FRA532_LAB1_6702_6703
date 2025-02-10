#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np

class CarLikeInvKinematics:
    def __init__(self, r: float):
        self.r = r  # Wheel radius

    def get_wheel_speed(self, linear_velocity: float):
        # Both drive wheels rotate at the same speed.
        wheel_speed = linear_velocity / self.r
        return np.array([wheel_speed, wheel_speed])


class SteeringController(Node):
    def __init__(self):
        super().__init__('steering_controller')
        
        # Robot parameters
        self.wheel_base = 0.2         # Distance between front and rear axles (meters)
        self.track_width = 0.14       # Distance between left and right wheels (meters)
        self.max_steering_angle = 0.523598767  # 30 degrees in radians
        self.wheel_radius = 0.045      # 4.5 cm
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

    def compute_ackermann_angles(self, steering_angle):
        """Calculate left and right steering angles using Ackermann geometry."""
        if steering_angle == 0:
            return 0.0, 0.0
        
        tan_steer = np.tan(steering_angle)
        
        # Standard formulas:
        left_angle = np.arctan(
            (self.wheel_base * tan_steer) / 
            (self.wheel_base - 0.5 * self.track_width * tan_steer)
        )
        right_angle = np.arctan(
            (self.wheel_base * tan_steer) / 
            (self.wheel_base + 0.5 * self.track_width * tan_steer)
        )
        
        # Apply steering angle limits
        left_angle = np.clip(left_angle, -self.max_steering_angle, self.max_steering_angle)
        right_angle = np.clip(right_angle, -self.max_steering_angle, self.max_steering_angle)
        
        return left_angle, right_angle

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x     # Linear velocity [m/s]
        w = msg.angular.z    # Angular velocity [rad/s]
        
        # Handle the case when the vehicle is (nearly) stationary.
        if abs(v) < 0.001:
            if w == 0.0:
                left_angle, right_angle = 0.0, 0.0
            else:
                # For testing, use the maximum steering angle in the direction of rotation.
                steering_angle = np.sign(w) * self.max_steering_angle
                left_angle, right_angle = self.compute_ackermann_angles(steering_angle)
        else:
            # Normal Ackermann steering calculation.
            if w == 0.0:
                steering_angle = 0.0
            else:
                # Compute the nominal steering angle using the bicycle model:
                # δ = arctan(wheel_base / R) with R = v / w.
                radius = v / w
                steering_angle = np.arctan(self.wheel_base / radius)
            
            left_angle, right_angle = self.compute_ackermann_angles(steering_angle)
        
        # --- Ackermann Check ---
        if (abs(np.tan(left_angle)) > 1e-6) and (abs(np.tan(right_angle)) > 1e-6):
            cot_left = 1.0 / np.tan(left_angle)
            cot_right = 1.0 / np.tan(right_angle)
            computed_diff = cot_left - cot_right
            ideal_diff = self.track_width / self.wheel_base
            percentage_ackermann = (computed_diff / ideal_diff) * 100.0
            # self.get_logger().info(
            #     f"Ackermann check: computed diff = {computed_diff:.3f}, ideal diff = {ideal_diff:.3f}, "
            #     f"percentage = {percentage_ackermann:.1f}%"
            # )
        
        self.publish_steering(left_angle, right_angle)

        # Use the car-like inverse kinematics for drive wheels.
        car_like_inv_k = CarLikeInvKinematics(self.wheel_radius)
        wheel_speeds = car_like_inv_k.get_wheel_speed(v)

        wheel_msg = Float64MultiArray()
        wheel_msg.data = [wheel_speeds[0], wheel_speeds[1]]
        self.pub_wheel_spd.publish(wheel_msg)

    def publish_steering(self, left_angle, right_angle):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['front_left_steering', 'front_right_steering']
        
        point = JointTrajectoryPoint()
        point.positions = [float(left_angle), float(right_angle)]
        point.time_from_start = rclpy.duration.Duration(seconds=0.5).to_msg()
        
        traj_msg.points.append(point)
        self.pub_steering.publish(traj_msg)
        self.get_logger().info(
            f"Steering command: left = {left_angle:.3f} rad, right = {right_angle:.3f} rad"
        )

def main(args=None):
    rclpy.init(args=args)
    node = SteeringController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
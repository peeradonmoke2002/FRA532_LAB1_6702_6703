#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
from rclpy.duration import Duration
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import yaml
import numpy as np
import math
import os

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.declare_parameter("mode", "noslip")
        self.mode = self.get_parameter("mode").get_parameter_value().string_value

        self.dt_loop = 1 / 100.0  # 0.01 s loop period

        # Robot parameters.
        self.wheel_base = 0.2         
        self.track_width = 0.14       
        self.max_steering_angle = 0.523598767  # 30° in radians
        self.wheel_radius = 0.045  

        # PID gains for steering.
        self.kp_steer = 0.25  # Increased for better response
        self.ki_steer = 0.01  # Small integral gain to reduce steady-state error
        self.kd_steer = 0.005  # Increased to reduce overshoot

        # PID gains for speed.
        self.kp_speed = 20.5  # Increased speed gain
        self.ki_speed = 10.05
        self.kd_speed = 0.05  # Small derivative gain to smooth speed control

        # Initialize PID state variables.
        self.integral_steer = 0.0
        self.prev_error_steer = 0.0
        self.integral_speed = 0.0
        self.prev_error_speed = 0.0

        # Load the path from YAML.
        self.waypoints = self.load_path("path.yaml")

        # Publishers.
        self.pub_steering = self.create_publisher(
            Float64MultiArray, 
            '/position_controllers/commands', 
            10
        )
        self.pub_wheel_spd = self.create_publisher(
            Float64MultiArray, 
            '/velocity_controllers/commands', 
            10
        )
        
        # Subscriber for Gazebo model states.
        # Note: We renamed the callback to model_states_callback.
        self.subscription_gazebo = self.create_subscription(
            ModelStates, 
            '/gazebo/model_states', 
            self.model_states_callback, 
            10
        )
        
    def load_path(self, filename):
        # If the filename is not absolute, look it up in the package share directory.
        if not os.path.isabs(filename):
            path_tracking_package = get_package_share_directory("path_tracking")
            filename = os.path.join(path_tracking_package, "path_data", filename)
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
        return [(wp['x'], wp['y'], wp.get('yaw', 0.0)) for wp in data]
        
    def nearest_waypoint(self, x, y, yaw):
        tolerance = 0.05  # Ignore waypoints closer than 5 cm
        forward_threshold = 0.1  # The waypoint must be in front of the robot
        min_distance = float('inf')
        best_index = None

        for i, (wx, wy, _) in enumerate(self.waypoints):
            dx = wx - x
            dy = wy - y
            distance = np.hypot(dx, dy)
            if distance < tolerance:
                self.get_logger().info(f"⚠️ Ignoring waypoint {i} (distance={distance:.3f} m, too close)")
                continue
            robot_dir = np.array([np.cos(yaw), np.sin(yaw)])
            wp_vector = np.array([dx, dy])
            dot_product = np.dot(robot_dir, wp_vector)
            if dot_product > forward_threshold and distance < min_distance:
                min_distance = distance
                best_index = i

        if best_index is None:
            distances = [np.hypot(wx - x, wy - y) for wx, wy, _ in self.waypoints]
            best_index = int(np.argmin(distances))
            self.get_logger().warn(f"⚠️ No valid forward waypoint found, selecting waypoint {best_index} (fallback)")
        else:
            self.get_logger().info(f"🎯 Selected waypoint {best_index}, distance={min_distance:.3f} m")
        return best_index

    def pid_control(self, error, prev_error, integral, kp, ki, kd, dt):
        derivative = (error - prev_error) / dt
        integral += error * dt
        return kp * error + ki * integral + kd * derivative, integral, error
    
    def compute_ackermann_angles(self, center_steer: float):
        if abs(center_steer) < 1e-6:
            return 0.0, 0.0
        sin_steer = np.sin(center_steer)
        cos_steer = np.cos(center_steer)
        inside = np.arctan(
            (2 * self.wheel_base * sin_steer) /
            (2 * self.wheel_base * cos_steer - self.track_width * sin_steer)
        )
        outside = np.arctan(
            (2 * self.wheel_base * sin_steer) /
            (2 * self.wheel_base * cos_steer + self.track_width * sin_steer)
        )
        return inside, outside

    def model_states_callback(self, msg):
        """
        Callback for Gazebo model states.
        This function finds the "limo" model, computes control errors,
        applies PID for steering and speed, and publishes commands.
        """
        try:    
            index = msg.name.index("limo")  # Ensure correct model name.
        except ValueError:
            self.get_logger().error("❌ Robot model not found in Gazebo!")
            return

        mode = self.mode  # Already fetched from parameters.
        pose = msg.pose[index]
        x, y = pose.position.x, pose.position.y
        _, _, yaw = self.quaternion_to_euler(pose.orientation)

        target_idx = self.nearest_waypoint(x, y, yaw)
        target_x, target_y, target_yaw = self.waypoints[target_idx]

        # Compute steering error.
        error_steer = np.arctan2(target_y - y, target_x - x) - yaw
        error_steer = (error_steer + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-pi, pi]
        # Compute speed error as the distance to the target.
        error_speed = np.hypot(target_x - x, target_y - y)

        steer, self.integral_steer, self.prev_error_steer = self.pid_control(
            error_steer, self.prev_error_steer, self.integral_steer,
            self.kp_steer, self.ki_steer, self.kd_steer, self.dt_loop)

        speed, self.integral_speed, self.prev_error_speed = self.pid_control(
            error_speed, self.prev_error_speed, self.integral_speed,
            self.kp_speed, self.ki_speed, self.kd_speed, self.dt_loop)

        # Clamp steering and speed.
        steer = np.clip(steer, -self.max_steering_angle, self.max_steering_angle)
        speed = max(2.5, min(speed, 8.0))  # Adjusted speed limits

        # Publish commands based on mode.
        if mode == "basic":
            self.publish_steering(steer, steer)
        elif mode == "noslip":
            turn_sign = np.sign(steer)
            if turn_sign > 0:
                left_angle, right_angle = self.compute_ackermann_angles(steer)
                self.publish_steering(left_angle, right_angle)
            else:
                right_angle, left_angle = self.compute_ackermann_angles(steer)
                self.publish_steering(left_angle, right_angle)
        else:
            self.get_logger().error("Error: mode not defined")
            return

        self.publish_wheel_speed(speed, speed)

        self.get_logger().info(f"🟢 Target WP: x={target_x:.3f}, y={target_y:.3f}")
        self.get_logger().info(f"🛞 Steering: {steer:.3f} rad, Speed: {speed:.3f} m/s")

    def publish_steering(self, steering_L: float, steering_R: float):
        steering_msg = Float64MultiArray()
        steering_msg.data = [steering_L, steering_R]
        self.pub_steering.publish(steering_msg)

    def publish_wheel_speed(self, wheelspeed_L: float, wheelspeed_R: float):
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [wheelspeed_L, wheelspeed_R]
        self.pub_wheel_spd.publish(wheel_msg)

    def quaternion_to_euler(self, q):
        """Convert quaternion to Euler angles (roll, pitch, yaw). Only yaw is used."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return 0.0, 0.0, yaw  # Return only yaw

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
import yaml
import numpy as np
import math
import os
from ament_index_python.packages import get_package_share_directory


class PIDBicycleController(Node):
    def __init__(self):
        super().__init__('pid_bicycle_controller')
        self.dt_loop = 1 / 50.0  # Loop time in seconds (50 Hz)

        self.wheel_base = 0.2      # Distance between front and rear axles (meters)
        self.wheel_radius = 0.045  # Rear wheel radius (meters)
        self.max_steering_angle = 0.523598767 / 2  # Limit steering angle

        # PID gains for steering
        self.kp_steer = 0.15  
        self.ki_steer = 0.01
        self.kd_steer = 0.005

        # PID gains for speed
        self.kp_speed = 5.5  
        self.ki_speed = 1.05
        self.kd_speed = 0.05  # Small derivative gain to smooth speed control

        self.integral_steer = 0.0
        self.prev_error_steer = 0.0
        self.integral_speed = 0.0
        self.prev_error_speed = 0.0

        self.min_speed = 2.5  # m/s
        self.max_speed = 8.0  # m/s

        # Load the path from YAML file (each waypoint: [x, y, yaw])
        self.waypoints = self.load_path("path.yaml")

        self.pub_steering = self.create_publisher(
            Float64MultiArray,
            "/position_controllers/commands",
            10
        )
        self.pub_wheel_spd = self.create_publisher(
            Float64MultiArray, '/velocity_controllers/commands', 10)

        # Subscribe to odometry from EKF on the topic '/odometry/filtered'
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

        # Create a timer callback to run the control loop at a fixed frequency
        self.control_timer = self.create_timer(self.dt_loop, self.timer_callback)

        # Variable to store the latest odometry
        self.robot_pose = None

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

    def odom_callback(self, msg: Odometry):
        """
        Callback for /odom messages.
        Updates the robot_pose from Odometry.
        """
        self.robot_pose = msg.pose.pose

    def timer_callback(self):
        """
        Timer callback at a fixed frequency (e.g., 50 Hz).
        Extracts the latest pose from self.robot_pose and calls the PID controller.
        """
        if self.robot_pose is None:
            return

        # Extract current state from robot_pose
        x = self.robot_pose.position.x
        y = self.robot_pose.position.y
        _, _, yaw = euler_from_quaternion([
            self.robot_pose.orientation.x,
            self.robot_pose.orientation.y,
            self.robot_pose.orientation.z,
            self.robot_pose.orientation.w
        ])

        # Call the controller step with current state
        self.controller_step(x, y, yaw)

    def controller_step(self, x, y, yaw):
        """
        Controller step:
          - Find the nearest waypoint.
          - Compute the heading error.
          - Compute the speed error based on no-slip condition: project error vector onto the vehicle's forward direction.
          - Compute PID outputs.
          - Publish commands.
        """
        target_idx = self.nearest_waypoint(x, y, yaw)
        target_x, target_y, target_yaw = self.waypoints[target_idx]

        # Compute the error vector from current position to target waypoint
        error_vec = np.array([target_x - x, target_y - y])
        
        # Heading error (steering error) computed as difference between target bearing and vehicle heading
        target_bearing = math.atan2(error_vec[1], error_vec[0])
        error_steer = target_bearing - yaw
        error_steer = (error_steer + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]

        # Under no-slip condition, assume the vehicle's velocity is aligned with its heading.
        # Therefore, the effective speed error is the projection of error_vec onto the forward direction.
        forward_direction = np.array([np.cos(yaw), np.sin(yaw)])
        error_speed = np.dot(error_vec, forward_direction)

        # Compute PID outputs for steering and speed.
        steer, self.integral_steer, self.prev_error_steer = self.pid_control(
            error_steer, self.prev_error_steer, self.integral_steer,
            self.kp_steer, self.ki_steer, self.kd_steer, self.dt_loop)
        speed, self.integral_speed, self.prev_error_speed = self.pid_control(
            error_speed, self.prev_error_speed, self.integral_speed,
            self.kp_speed, self.ki_speed, self.kd_speed, self.dt_loop)

        # Clamp commands to limits.
        steer = np.clip(steer, -self.max_steering_angle, self.max_steering_angle)
        speed = np.clip(speed, self.min_speed, self.max_speed)

        self.publish_steering(steer, steer)
        self.publish_wheel_speed(speed)

        self.get_logger().info(f"🟢 Target WP: x={target_x:.3f}, y={target_y:.3f}")
        self.get_logger().info(f"🛞 Steering: {steer:.3f} rad, Speed: {speed:.3f} m/s")

    def publish_steering(self, front_left_steering: float, front_right_steering: float):
        steering_msg = Float64MultiArray()
        steering_msg.data = [front_left_steering, front_right_steering]
        self.pub_steering.publish(steering_msg)
        self.get_logger().info(f"🔄 Steering Published: L={front_left_steering:.3f} rad, R={front_right_steering:.3f} rad")

    def publish_wheel_speed(self, speed):
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [speed, speed]
        self.pub_wheel_spd.publish(wheel_msg)
        self.get_logger().info(f"⚡ Wheel Speed Published: {speed:.3f} m/s")

def euler_from_quaternion(quat):
    """
    Convert quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw).
    """
    x, y, z, w = quat
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(sinp) if abs(sinp) <= 1.0 else math.copysign(math.pi/2, sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = PIDBicycleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

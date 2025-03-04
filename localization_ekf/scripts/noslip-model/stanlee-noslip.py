#!/usr/bin/python3

import os
import math
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from ament_index_python.packages import get_package_share_directory

class StanleyNode(Node):
    def __init__(self):
        super().__init__('stanley_node')

        self.declare_parameter("mode", "noslip")
        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        
        # Subscribe to filtered odometry instead of Gazebo model states.
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )
        
        # Timer for running the controller periodically
        self.dt_loop = 1/100.0 
        self.create_timer(self.dt_loop, self.timer_callback)  

        # Publishers for steering and velocity commands
        self.steering_pub = self.create_publisher(
            Float64MultiArray,
            "/position_controllers/commands",
            10
        )
        self.velocity_pub = self.create_publisher(
            Float64MultiArray,
            "/velocity_controllers/commands",
            10
        )

        # Load the path from YAML; each row: [x, y]
        path_tracking_package = get_package_share_directory("path_tracking")
        path = self.load_path(f'{path_tracking_package}/path_data/path.yaml')
        self.path = np.array(path)[:, 0:2]
        if self.path is not None:
            self.get_logger().info(f"Loaded path with {self.path.shape[0]} points.")
        else:
            self.get_logger().error("Failed to load path!")
        
        # Compute yaw for each waypoint from differences (for path tangent)
        self.path_yaw = self.compute_path_yaw(self.path)
        # Combine into a full path: [x, y, yaw]
        self.path_full = np.hstack((self.path, self.path_yaw.reshape(-1, 1)))
        # self.publish_path()

        # Robot parameters.
        self.wheel_base = 0.2         
        self.track_width = 0.14       
        self.max_steering_angle = 0.523598767  
        self.wheel_radius = 0.045   
        # Vehicle parameters.
        self.L = self.wheel_base / 2   
        self.k = 1.0   # Stanley gain
        self.speed = 0.5  # Constant speed [m/s]
        self.lastFoundIndex = 0  # Last target waypoint index

        # State variables updated from odometry.
        self.currentPos = [0.0, 0.0]
        self.currentHeading = 0.0  # in degrees

        # Storage for the robot state (from odometry).
        self.robot_pose = None

    def load_path(self, file_path):
        """Load waypoints from a YAML file."""
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
        waypoints = [(wp['x'], wp['y'], wp.get('yaw', 0.0)) for wp in data]
        self.get_logger().info(f"Loaded {len(waypoints)} waypoints.")
        return waypoints

    def compute_path_yaw(self, path):
        """
        Computes yaw (in radians) for each waypoint based on the difference to the next.
        For the last waypoint, the yaw is copied from the previous one.
        """
        n = path.shape[0]
        yaws = np.zeros(n)
        for i in range(n - 1):
            dx = path[i+1, 0] - path[i, 0]
            dy = path[i+1, 1] - path[i, 1]
            yaws[i] = math.atan2(dy, dx)
        yaws[-1] = yaws[-2] if n > 1 else 0.0
        return yaws

    def normalize_angle(self, angle):
        """Normalize angle to the range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def stanley_controller_step(self, cx, cy, cyaw, currentPos, currentHeading, last_target_idx, speed):
        """
        Stanley controller step calculation.
        
        Parameters:
            cx, cy, cyaw: arrays of path x, y, and yaw values.
            currentPos: [x, y] current position.
            currentHeading: current heading in degrees.
            last_target_idx: index of the last target point.
            speed: current speed (m/s)
        
        Returns:
            delta: computed steering angle (rad)
            target_idx: updated target index
            error_front_axle: cross-track error
        """
        # Convert current heading to radians.
        yaw = math.radians(currentHeading)
        # Compute front axle position.
        fx = currentPos[0] + self.L * math.cos(yaw)
        fy = currentPos[1] + self.L * math.sin(yaw)
        
        # Compute distances from front axle to each waypoint.
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]
        d = np.hypot(dx, dy)
        target_idx = int(np.argmin(d))
        
        # Compute cross-track error (projection on the vector perpendicular to heading).
        error_x = cx[target_idx] - fx
        error_y = cy[target_idx] - fy
        error_front_axle = error_x * (-math.sin(yaw)) + error_y * math.cos(yaw)
        
        # Compute heading error between the path tangent and vehicle heading.
        theta_e = self.normalize_angle(cyaw[target_idx] - yaw)
        # Cross-track correction term.
        theta_d = math.atan2(self.k * error_front_axle, speed)
        
        # Total steering command.
        delta = theta_e + theta_d
        
        return delta, target_idx, error_front_axle

    def stanley_controller(self):
        """
        Computes the Stanley control action and commands the vehicle.
        """
        cx = self.path_full[:, 0]
        cy = self.path_full[:, 1]
        cyaw = self.path_full[:, 2]
        
        if self.robot_pose is None:
            self.get_logger().warn("No robot state received yet.")
            return
        
        # Extract current state from the odometry message.
        x = self.robot_pose.position.x
        y = self.robot_pose.position.y
        orientation_q = self.robot_pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.currentPos = [x, y]
        self.currentHeading = math.degrees(yaw)
        
        delta, target_idx, cte = self.stanley_controller_step(
            cx, cy, cyaw, self.currentPos, self.currentHeading, self.lastFoundIndex, self.speed)
        
        # Ensure target index does not go backward.
        if target_idx < self.lastFoundIndex:
            target_idx = self.lastFoundIndex
        self.lastFoundIndex = target_idx
        
        self.get_logger().info(
            f"Steering Angle: {delta:.2f} rad, TargetIdx: {target_idx}, Cross-Track Error: {cte:.2f}"
        )
        
        self.cmd_vel_steering(self.speed, delta)

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

    def cmd_vel_steering(self, vx, steering_angle):
        """
        Converts desired steering angle and speed into commands.
        Publishes:
          - Steering command as a Float64MultiArray (position control)
          - Velocity command as a Float64MultiArray (velocity control)
        """
        mode = self.mode
        
        # Convert linear velocity to wheel speed using the wheel radius.
        wheel_speed = vx / self.wheel_radius  
        
        # Publish steering commands based on selected mode.
        if mode == "basic":
            self.set_steering_angle(steering_angle, steering_angle)
        elif mode == "noslip":
            turn_sign = np.sign(steering_angle)
            if turn_sign > 0:
                left_angle, right_angle = self.compute_ackermann_angles(steering_angle)
                self.set_steering_angle(left_angle, right_angle)
            else:
                right_angle, left_angle = self.compute_ackermann_angles(steering_angle)
                self.set_steering_angle(left_angle, right_angle)
        else:
            self.get_logger().error("Error: mode not defined")
            return
        # Publish velocity command (same speed for both rear wheels).
        self.set_velocity(wheel_speed, wheel_speed)
    
    def timer_callback(self):
        """
        Periodic timer callback that runs the Stanley controller.
        """
        self.stanley_controller()
    
    def odom_callback(self, msg: Odometry):
        """
        Callback for /odometry/filtered.
        Updates the robot state from odometry.
        """
        self.robot_pose = msg.pose.pose
    
    def set_steering_angle(self, left_angle, right_angle):
        msg = Float64MultiArray()
        msg.data = [float(left_angle), float(right_angle)]
        self.steering_pub.publish(msg)
    
    def set_velocity(self, left_speed, right_speed):
        msg = Float64MultiArray()
        msg.data = [float(left_speed), float(right_speed)]
        self.velocity_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StanleyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from rclpy.duration import Duration
import tf_transformations
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
import numpy as np
import math
import yaml

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        self.declare_parameter("mode", "noslip")
        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        
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
        
        # Subscriber for odom-filtered.
        self.subscription_odom = self.create_subscription(
            Odometry, 
            '/odometry/filtered', 
            self.odom_callback, 
            10
        )
        # Timer for the control loop.
        self.dt_loop = 1/100
        self.timer = self.create_timer(self.dt_loop, self.pure_pursuit_control)
        
        # Load waypoints.
        path_tracking_package = get_package_share_directory("path_tracking")
        wp = self.load_waypoints(f'{path_tracking_package}/path_data/path.yaml')
        wp = np.array(wp)  
        self.waypoints = wp[:, 0:2]

        # State variables.
        self.current_index = 0
        self.lookahead_distance = 1.0  # meters
        self.position = (0.0, 0.0)
        self.yaw = 0.0
        self.theta = 0.0  # yaw from odom (in radians)
        self.x = 0.0
        self.y = 0.0
        self.robot_path = []  # To store the tracked path.
        self.prev_position = None

        # Robot parameters.
        self.wheel_base = 0.2         
        self.track_width = 0.14       
        self.max_steering_angle = 0.523598767  
        self.wheel_radius = 0.045     

    def load_waypoints(self, file_path):
        """Load waypoints from a YAML file."""
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
        waypoints = [(wp['x'], wp['y'], wp.get('yaw', 0.0)) for wp in data]
        self.get_logger().info(f"Loaded {len(waypoints)} waypoints.")
        return waypoints
    
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

    def odom_callback(self, msg):
        """Update the robot's position and orientation from the filtered odometry."""
        pose = msg.pose.pose
        self.position = (pose.position.x, pose.position.y)
        orientation_q = pose.orientation
        # Convert quaternion to Euler angles.
        _, _, self.theta = tf_transformations.euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        self.yaw = self.theta  # For consistency, set yaw equal to theta.

    def publish_steering(self, steering_L: float, steering_R: float):
        steering_msg = Float64MultiArray()
        steering_msg.data = [steering_L, steering_R]
        self.pub_steering.publish(steering_msg)

    def publish_wheel_speed(self, wheelspeed_L: float, wheelspeed_R: float):
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [wheelspeed_L, wheelspeed_R]
        self.pub_wheel_spd.publish(wheel_msg)

    def pure_pursuit_control(self):
        mode = self.get_parameter("mode").get_parameter_value().string_value
        if self.current_index >= len(self.waypoints):
            self.current_index = 0 
            self.get_logger().info("Reached final waypoint. Restarting path.")
            return
        
        # Update current position.
        self.x = self.position[0]
        self.y = self.position[1]
        if (self.x, self.y) != (0.0, 0.0):
            self.robot_path.append((self.x, self.y))
        
        # Calculate lookahead point in global coordinates.
        odom_lookahead_x = self.x + (self.lookahead_distance * math.cos(self.theta))
        odom_lookahead_y = self.y + (self.lookahead_distance * math.sin(self.theta))
        lookahead_point = (odom_lookahead_x, odom_lookahead_y)
        
        # Find the waypoint closest to the lookahead point.
        distances = np.sqrt(np.sum((self.waypoints - np.array([odom_lookahead_x, odom_lookahead_y])) ** 2, axis=1))
        min_dist_index = np.argmin(distances)
        wp_x, wp_y = self.waypoints[min_dist_index]

        # Compute error in global frame.
        dx = wp_x - self.x
        dy = wp_y - self.y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Check if within lookahead distance.
        if distance < self.lookahead_distance:
            if self.prev_position is not None:
                movement = math.sqrt((self.x - self.prev_position[0])**2 + (self.y - self.prev_position[1])**2)
                if movement > 0.01:
                    self.get_logger().info(f"Reached waypoint {self.current_index + 1}")
                    self.current_index += 1
                    self.prev_position = (self.x, self.y)
                    return
            else:
                self.prev_position = (self.x, self.y)
        
        # Transform (dx, dy) into the robot's local frame.
        transform_x = dx * math.cos(self.theta) + dy * math.sin(self.theta)
        transform_y = -dx * math.sin(self.theta) + dy * math.cos(self.theta)
        
        # Pure pursuit curvature calculation.
        curvature = 0.0
        if distance != 0:
            curvature = 2 * transform_y / (distance**2)
        steering_angle = math.atan(self.wheel_base * curvature)
        
        # Publish steering commands based on selected mode.
        if mode == "basic":
            self.publish_steering(steering_angle, steering_angle)
        elif mode == "noslip":
            turn_sign = np.sign(curvature)
            if turn_sign > 0:
                left_angle, right_angle = self.compute_ackermann_angles(steering_angle)
                self.publish_steering(left_angle, right_angle)
            else:
                right_angle, left_angle = self.compute_ackermann_angles(steering_angle)
                self.publish_steering(left_angle, right_angle)
        else:
            self.get_logger().error("Error: mode not defined")
            return
        
        # Publish wheel speed.
        speed = 0.5
        wheel_speed = min(speed, distance) / self.wheel_radius
        self.publish_wheel_speed(wheel_speed, wheel_speed)
        

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Pure Pursuit node.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

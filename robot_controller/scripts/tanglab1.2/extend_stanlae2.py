#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
import numpy as np
import math
import yaml

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class ExtendedStanleyController(Node):
    def __init__(self):
        super().__init__('extended_stanley_controller')
        self.dt_loop = 1 / 50.0

        # Robot parameters
        self.wheel_base = 0.2
        self.track_width = 0.13
        self.front_offset = 0.1

        # Control gains
        self.k_stanley = 1.0
        self.k_soft = 0.5
        self.kd_yaw = 0.5
        self.kd_steer = 0.5
        self.delta_max = 0.7  # ~40°, adjust per robot
        self.desired_speed = 0.5  # Slower for testing

        self.prev_steering = 0.0
        self.waypoints = self.load_path('/home/tang/ros2_lab1_m/src/FRA532_LAB1_6702_6703/robot_controller/data/path.yaml')
        self.current_waypoint_index = 0

        # Publishers
        self.pub_steering = self.create_publisher(JointTrajectory, '/joint_trajectory_position_controller/joint_trajectory', 10)
        self.pub_wheel_spd = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)

        # Subscriber
        self.create_subscription(ModelStates, '/gazebo/model_states', self.gazebo_callback, 10)

        # Latest state
        self.latest_pose = None
        self.latest_twist = None

    def load_path(self, file_path):
        try:
            with open(file_path, 'r') as file:
                data = yaml.safe_load(file)
            if 'path' in data:
                self.get_logger().info(f"Loaded {len(data['path'])} waypoints")
                return data['path']
            else:
                self.get_logger().error("Key 'path' not found!")
                return []
        except Exception as e:
            self.get_logger().error(f"Failed to load path: {e}")
            return []

    def gazebo_callback(self, msg):
        try:
            index = msg.name.index("limo")
            self.latest_pose = msg.pose[index]
            self.latest_twist = msg.twist[index]
            self.control_loop()
        except ValueError:
            self.get_logger().error("Robot 'limo' not found in Gazebo!")

    def control_loop(self):
        if self.latest_pose is None or self.latest_twist is None:
            return

        x, y = self.latest_pose.position.x, self.latest_pose.position.y
        _, _, yaw = self.quaternion_to_euler(self.latest_pose.orientation)
        front_x = x + self.front_offset * math.cos(yaw)
        front_y = y + self.front_offset * math.sin(yaw)
        v = np.hypot(self.latest_twist.linear.x, self.latest_twist.linear.y)
        yaw_rate = self.latest_twist.angular.z

        target = self.get_target_waypoint(front_x, front_y, yaw)
        if target is None:
            self.get_logger().info("No more waypoints")
            return

        target_x, target_y, target_yaw = target
        steering = self.compute_extended_stanley(front_x, front_y, yaw, v, yaw_rate, target_x, target_y, target_yaw, 0.0)
        self.get_logger().info(f"Steering: {steering:.3f}, Waypoint: {self.current_waypoint_index}")
        self.publish_steering(steering)
        self.publish_wheel_speed(self.desired_speed)

    def get_target_waypoint(self, front_x, front_y, yaw):
            while self.current_waypoint_index < len(self.waypoints):
                wp = self.waypoints[self.current_waypoint_index]
                target_x, target_y, target_yaw = wp['x'], wp['y'], wp['yaw']
                dx = target_x - front_x
                dy = target_y - front_y
                dist = math.hypot(dx, dy)
                heading_vector = [math.cos(yaw), math.sin(yaw)]
                dot = dx * heading_vector[0] + dy * heading_vector[1]
                if dot < 0 or dist < 0.2:  # Increased threshold
                    self.current_waypoint_index += 1
                    self.get_logger().info(f"Skipping waypoint {self.current_waypoint_index - 1}, Dist: {dist:.3f}")
                    continue
                return target_x, target_y, target_yaw
            return None

    def compute_extended_stanley(self, x, y, yaw, v, yaw_rate, target_x, target_y, target_yaw, target_yaw_rate):
        dx = target_x - x
        dy = target_y - y
        cross_track_error = dx * (-math.sin(target_yaw)) + dy * math.cos(target_yaw)
        heading_error = normalize_angle(yaw - target_yaw)
        stanley_term = math.atan2(self.k_stanley * cross_track_error, self.k_soft + max(v, 0.1))  # Avoid division by zero
        yaw_damping = self.kd_yaw * (target_yaw_rate - yaw_rate)
        steering_est = heading_error + stanley_term + yaw_damping
        steer_damping = self.kd_steer * (self.prev_steering - steering_est)
        steering_cmd = steering_est + steer_damping
        steering_cmd = np.clip(steering_cmd, -self.delta_max, self.delta_max)
        self.get_logger().info(f"CTE: {cross_track_error:.3f}, HE: {heading_error:.3f}, Steering: {steering_cmd:.3f}")
        self.prev_steering = steering_cmd
        return steering_cmd

    def publish_steering(self, steering):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['front_left_steering', 'front_right_steering']
        point = JointTrajectoryPoint()
        point.positions = [steering, steering]
        traj_msg.points.append(point)
        self.pub_steering.publish(traj_msg)

    def publish_wheel_speed(self, speed):
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [speed, speed]
        self.pub_wheel_spd.publish(wheel_msg)

    def quaternion_to_euler(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return 0.0, 0.0, yaw

def main(args=None):
    rclpy.init(args=args)
    node = ExtendedStanleyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
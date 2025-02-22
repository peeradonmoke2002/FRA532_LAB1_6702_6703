#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
from rclpy.duration import Duration
import yaml
import numpy as np
import math

def normalize_angle(angle):
    """Normalize an angle to the range [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi

class StanleyBicycleController(Node):
    def __init__(self):
        super().__init__('stanley_bicycle_controller')
        self.dt_loop = 1 / 50.0  # Loop time in seconds (50 Hz)

        # ✅ Robot parameters
        self.wheel_base = 0.2      # Distance between front and rear axles (meters)
        self.wheel_radius = 0.045  # Rear wheel radius (meters)
        self.max_steering_angle = 0.523598767/2   # Limit steering angle (radians)
        self.front_offset = 0.1

        # ✅ Stanley Control Gain (Tunable)
        self.k_stanley = 6.5  # Gain for cross-track error

        # ✅ Desired Speed (m/s)
        self.desired_speed = 6.0

        # ✅ Speed controller gain (P controller)
        self.kp_speed = 55.0  # Tuning parameter for speed control

        # ✅ Load waypoints (expecting x, y, yaw in each waypoint)
        self.waypoints = self.load_path('/home/tang/ros2_lab1_m/src/FRA532_LAB1_6702_6703/robot_controller/data/path.yaml')

        # ✅ Publishers
        self.pub_steering = self.create_publisher(
            JointTrajectory, '/joint_trajectory_position_controller/joint_trajectory', 10)
        self.pub_wheel_spd = self.create_publisher(
            Float64MultiArray, '/velocity_controllers/commands', 10)
        
        # ✅ Subscriber to Gazebo model states
        self.create_subscription(ModelStates, '/gazebo/model_states', self.gazebo_callback, 10)

    def load_path(self, filename):
        """
        Load waypoints from a YAML file.
        Expecting each waypoint to have keys: x, y, yaw.
        """
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
        if 'path' not in data:
            self.get_logger().error(f"Key 'path' not found in {filename}. Check file formatting.")
            return []
        return [(point['x'], point['y'], point['yaw']) for point in data['path']]

    def nearest_waypoint(self, x, y, yaw):
        """
        Select the nearest waypoint that is in front of the vehicle.
        Only consider waypoints for which the dot product between the vehicle's heading
        vector and the vector from the vehicle to the waypoint is positive.
        Returns the index of the best waypoint or None if no forward waypoint is found.
        """
        tolerance = 0.05  # Ignore waypoints closer than 5 cm
        forward_waypoints = []

        # Vehicle's heading vector
        heading_vec = np.array([math.cos(yaw), math.sin(yaw)])
        current_pos = np.array([x, y])

        for i, (wx, wy, _) in enumerate(self.waypoints):
            waypoint_pos = np.array([wx, wy])
            diff = waypoint_pos - current_pos
            distance = np.linalg.norm(diff)
            if distance < tolerance:
                continue  # Ignore very close waypoints
            # Check if the waypoint is in front
            if np.dot(diff, heading_vec) > 0:
                forward_waypoints.append((i, distance))

        if forward_waypoints:
            # Select the waypoint with the smallest distance among forward waypoints
            best_index = min(forward_waypoints, key=lambda tup: tup[1])[0]
            self.get_logger().info(f"Selected forward waypoint {best_index}")
            return best_index
        else:
            self.get_logger().warn("No forward waypoint found. Not updating control command.")
            return None

    def gazebo_callback(self, msg):
        """
        Process Gazebo model state and compute control commands using Stanley Control.
        """
        try:
            index = msg.name.index("limo")  # Ensure correct model name
        except ValueError:
            self.get_logger().error("Robot model not found in Gazebo!")
            return

        # Get current pose and twist
        pose = msg.pose[index]
        twist = msg.twist[index]
        x, y = pose.position.x, pose.position.y
        _, _, yaw = self.quaternion_to_euler(pose.orientation)

        # Compute front axle center (control point)
        front_x = x + self.front_offset * math.cos(yaw)
        front_y = y + self.front_offset * math.sin(yaw)


        # Compute current speed from twist data
        vx = twist.linear.x
        vy = twist.linear.y
        v = np.hypot(vx, vy)

        # Find the nearest forward waypoint using the front axle position
        target_idx = self.nearest_waypoint(front_x, front_y, yaw)
        if target_idx is None:
            # Do not update control command if no forward waypoint is found.
            return

        target_x, target_y, target_yaw = self.waypoints[target_idx]

        # Compute Heading Error: θ_e = θ - θ_p, normalized to [-π, π]
        heading_error = normalize_angle(target_yaw - yaw)

        # Compute Cross-Track Error using front axle position:
        # e_f = ((front_x, front_y) - (target_x, target_y)) dot (-sin(target_yaw), cos(target_yaw))
        cross_track_error = (front_x - target_x) * (-math.sin(target_yaw)) + (front_y - target_y) * math.cos(target_yaw)

        # Stanley Control Law: δ = θ_e + arctan( (k * e_f) / (v + ε) )
        epsilon = 1e-3  # Avoid division by zero
        steer = heading_error + math.atan2(self.k_stanley * cross_track_error, v + epsilon)
        steer = normalize_angle(steer)
        steer = np.clip(steer, -self.max_steering_angle, self.max_steering_angle)

        # Speed control using a simple P controller:
        # speed_command = current_speed + kp * (desired_speed - current_speed) * dt
        speed_error = self.desired_speed - v
        speed_command = v + self.kp_speed * speed_error * self.dt_loop
        # Optionally, you can add limits on acceleration/deceleration if needed.

        # Publish Commands
        self.publish_steering(steer)
        self.publish_wheel_speed(speed_command)

        self.get_logger().info(f"Stanley Control: Target WP: x={target_x:.3f}, y={target_y:.3f}")
        self.get_logger().info(f"Steering: {steer:.3f} rad, Speed: {speed_command:.3f} m/s")

    def compute_steering(self, x, y, yaw, v):
        """ Compute Stanley steering angle """
        if self.target_index >= len(self.waypoints):
            return 0.0  # หยุดเมื่อถึงเป้าหมาย

        target_x = self.waypoints[self.target_index]['x']
        target_y = self.waypoints[self.target_index]['y']
        target_yaw = self.waypoints[self.target_index]['yaw']

        # ✅ คำนวณพิกัดล้อหน้า
        front_x = x + self.front_offset * math.cos(yaw)
        front_y = y + self.front_offset * math.sin(yaw)

        # ✅ คำนวณ Cross-Track Error โดยใช้ (front_x, front_y)
        dx = target_x - front_x
        dy = target_y - front_y
        cross_track_error = (dx * -math.sin(target_yaw)) + (dy * math.cos(target_yaw))

        # ✅ คำนวณ Heading Error (เปลี่ยนสัญลักษณ์เป็น target - current)
        heading_error = normalize_angle(target_yaw - yaw)

        # ✅ คำนวณ Stanley Control Law
        epsilon = 1e-3  # ป้องกันหารด้วย 0
        steering_angle = heading_error + math.atan2(self.k_stanley * cross_track_error, v + epsilon)

        # ✅ จำกัดมุมเลี้ยวให้อยู่ในช่วงที่รถรองรับ
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)

        # ✅ อัปเดต index ของ waypoint ถ้าใกล้เป้าหมายมากพอ
        if np.hypot(dx, dy) < 0.5:  # ปรับ threshold ได้
            self.target_index += 1

        return steering_angle


    def publish_steering(self, steering):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['front_left_steering', 'front_right_steering']
        point = JointTrajectoryPoint()
        point.positions = [steering, steering]
        point.time_from_start = Duration(seconds=0.1).to_msg()
        traj_msg.points.append(point)
        self.pub_steering.publish(traj_msg)

    def publish_wheel_speed(self, speed):
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [speed, speed]
        self.pub_wheel_spd.publish(wheel_msg)

    def quaternion_to_euler(self, q):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return 0.0, 0.0, yaw  # Only return yaw for our use

def main(args=None):
    rclpy.init(args=args)
    node = StanleyBicycleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

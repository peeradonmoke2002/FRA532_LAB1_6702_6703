#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
import yaml
import numpy as np
from rclpy.duration import Duration

class PIDBicycleController(Node):
    def __init__(self):
        super().__init__('pid_bicycle_controller')
        self.dt_loop = 1 / 50.0  # Loop time in seconds (50 Hz)

        # ✅ Robot parameters
        self.wheel_base = 0.2  # Distance between front and rear axles (meters)
        self.wheel_radius = 0.045  # Wheel radius (meters)
        self.max_steering_angle = 0.523598767 / 3  # Limit steering angle

        # ✅ PID Gains
        self.kp_steer = 1
        self.ki_steer = 0
        self.kd_steer = 0.1
        self.kp_speed = 1
        self.ki_speed = 0
        self.kd_speed = 0

        # ✅ PID Variables
        self.integral_steer = 0.0
        self.prev_error_steer = 0.0
        self.integral_speed = 0.0
        self.prev_error_speed = 0.0

        # ✅ Track visited waypoints 
        self.visited_points = set()

        # ✅ Load waypoints from file
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
        """
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
        if 'path' not in data:
            self.get_logger().error(f"Key 'path' not found in {filename}.")
            return []
        return [(point['x'], point['y'], point['yaw']) for point in data['path']]

    def nearest_waypoint(self, x, y, yaw):
        """
        Select the nearest waypoint that is in front of the robot.
        - Ignore waypoints that are too close (within 5 cm).
        - Choose waypoints with a forward direction based on the dot product.
        """
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

    def angle_difference(self, target_yaw, current_yaw):
        """
        Compute the angle difference between target and current yaw.
        Ensures the value is within [-pi, pi] range.
        """
        diff = target_yaw - current_yaw
        while diff > np.pi:
            diff -= 2 * np.pi
        while diff < -np.pi:
            diff += 2 * np.pi
        return diff

    def pid_control(self, error, prev_error, integral, kp, ki, kd, dt):
        """
        Compute PID control output.
        """
        derivative = (error - prev_error) / dt
        integral += error * dt
        return kp * error + ki * integral + kd * derivative, integral, error

    def pure_pursuit_steering(self, x, y, yaw):
        """
        Compute Pure Pursuit steering angle.
        """
        lookahead_distance = 1.5  # Lookahead distance (tunable)
        lookahead_point = None

        for (wx, wy, _) in self.waypoints:
            distance = np.hypot(wx - x, wy - y)
            if distance >= lookahead_distance:
                lookahead_point = (wx, wy)
                break

        if lookahead_point is None:
            # Fallback: use the last waypoint
            lookahead_point = (self.waypoints[-1][0], self.waypoints[-1][1])

        angle_to_point = np.arctan2(lookahead_point[1] - y, lookahead_point[0] - x)
        alpha = angle_to_point - yaw
        delta_pp = np.arctan2(2 * self.wheel_base * np.sin(alpha), lookahead_distance)
        return delta_pp

    def gazebo_callback(self, msg):
        """
        Main callback function for processing Gazebo model state.
        """
        try:
            index = msg.name.index("limo")  # Replace "limo" with your robot's model name in Gazebo
        except ValueError:
            self.get_logger().error("❌ Robot model not found in Gazebo!")
            return

        pose = msg.pose[index]
        x, y = pose.position.x, pose.position.y
        _, _, yaw = self.quaternion_to_euler(pose.orientation)

        # Select the nearest waypoint
        target_idx = self.nearest_waypoint(x, y, yaw)
        target_x, target_y, target_yaw = self.waypoints[target_idx]

        # Compute error distance
        error_speed = np.hypot(target_x - x, target_y - y)
        distance_threshold = 1.5  # If distance > threshold, use Pure Pursuit

        # Compute Pure Pursuit steering term
        pp_steering = self.pure_pursuit_steering(x, y, yaw)
        self.get_logger().info(f"Feedforward (Pure Pursuit) term: {pp_steering:.3f} rad")

        if error_speed > distance_threshold:
            steering_cmd = pp_steering
            self.get_logger().info("🔵 Using Pure Pursuit due to large distance.")
        else:
            # Compute PID steering
            error_steer = np.arctan2(target_y - y, target_x - x) - yaw
            steer_pid, self.integral_steer, self.prev_error_steer = self.pid_control(
                error_steer, self.prev_error_steer, self.integral_steer,
                self.kp_steer, self.ki_steer, self.kd_steer, self.dt_loop)
            steering_cmd = steer_pid + pp_steering

        # Limit steering within allowed range
        steering_cmd = np.clip(steering_cmd, -self.max_steering_angle, self.max_steering_angle)

        # Compute speed PID
        speed, self.integral_speed, self.prev_error_speed = self.pid_control(
            error_speed, self.prev_error_speed, self.integral_speed,
            self.kp_speed, self.ki_speed, self.kd_speed, self.dt_loop)
        speed = np.clip(speed, 5.0, 10.0)

        self.publish_steering(steering_cmd)
        self.publish_wheel_speed(speed)

    def publish_steering(self, steering):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['front_left_steering', 'front_right_steering']
        point = JointTrajectoryPoint()
        point.positions = [steering, steering]
        self.pub_steering.publish(traj_msg)

    def publish_wheel_speed(self, speed):
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [speed, speed]
        self.pub_wheel_spd.publish(wheel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDBicycleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

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
import os

class PIDBicycleController(Node):
    def __init__(self):
        super().__init__('pid_bicycle_controller')
        self.dt_loop = 1 / 50.0  # Loop time in seconds (50 Hz)


        self.wheel_base = 0.2  # Distance between front and rear axles (meters)
        self.wheel_radius = 0.045  # Rear wheel radius (meters)
        self.max_steering_angle = 0.523598767 / 2  # Limit steering angle

        self.kp_steer = 0.25  # Increased for better response
        self.ki_steer = 0.01  # Small integral gain to reduce steady-state error
        self.kd_steer = 0.005  # Increased to reduce overshoot
        self.kp_speed = 20.5  # Increased speed gain
        self.ki_speed = 10.05
        self.kd_speed = 0.05  # Small derivative gain to smooth speed control

        self.integral_steer = 0.0
        self.prev_error_steer = 0.0
        self.integral_speed = 0.0
        self.prev_error_speed = 0.0

        self.min_speed = 2.5  # m/s
        self.max_speed = 8.0  # m/s


        self.waypoints = self.load_path("path.yaml")

        self.pub_steering = self.create_publisher(
            JointTrajectory, '/joint_trajectory_position_controller/joint_trajectory', 10)
        self.pub_wheel_spd = self.create_publisher(
            Float64MultiArray, '/velocity_controllers/commands', 10)
        

        self.create_subscription(ModelStates, '/gazebo/model_states', self.gazebo_callback, 10)

    def load_path(self, filename):
        if not os.path.isabs(filename):
            ros_workspace = os.getenv("ROS_WORKSPACE")
            if ros_workspace is None:
                script_dir = os.path.dirname(os.path.realpath(__file__))
                ros_workspace = script_dir.split('/src/')[0]

            filename = os.path.join(ros_workspace, "src", "FRA532_LAB1_6702_6703", "path_tracking", "data", filename)
        
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
        
        return [(point['x'], point['y'], point['yaw']) for point in data['path']]
    


    def pid_control(self, error, prev_error, integral, kp, ki, kd, dt):

        derivative = (error - prev_error) / dt
        integral += error * dt
        return kp * error + ki * integral + kd * derivative, integral, error

    def gazebo_callback(self, msg):

        try:
            index = msg.name.index("limo")  # Ensure correct model name
        except ValueError:
            self.get_logger().error("❌ Robot model not found in Gazebo!")
            return

        pose = msg.pose[index]
        x, y = pose.position.x, pose.position.y
        _, _, yaw = self.quaternion_to_euler(pose.orientation)

        target_idx = self.nearest_waypoint(x, y, yaw)
        target_x, target_y, target_yaw = self.waypoints[target_idx]


        error_steer = np.arctan2(target_y - y, target_x - x) - yaw
        error_steer = (error_steer + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-pi, pi]
        error_speed = np.hypot(target_x - x, target_y - y)

        steer, self.integral_steer, self.prev_error_steer = self.pid_control(
            error_steer, self.prev_error_steer, self.integral_steer,
            self.kp_steer, self.ki_steer, self.kd_steer, self.dt_loop)

        speed, self.integral_speed, self.prev_error_speed = self.pid_control(
            error_speed, self.prev_error_speed, self.integral_speed,
            self.kp_speed, self.ki_speed, self.kd_speed, self.dt_loop)

        steer = np.clip(steer, -self.max_steering_angle, self.max_steering_angle)
        speed = max(2.5, min(speed, 8.0))  # Adjusted speed limits


        self.publish_steering(steer)
        self.publish_wheel_speed(speed)

        self.get_logger().info(f"🟢 Target WP: x={target_x:.3f}, y={target_y:.3f}")
        self.get_logger().info(f"🛞 Steering: {steer:.3f} rad, Speed: {speed:.3f} m/s")

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
        """ Convert quaternion to Euler angles """
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return 0.0, 0.0, yaw  # Only return yaw

def main(args=None):
    rclpy.init(args=args)
    node = PIDBicycleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

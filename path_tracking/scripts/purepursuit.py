#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
import yaml
import numpy as np
import math
import matplotlib.pyplot as plt
import os

# Constants
k = 0.1       # Look forward gain
Lfc = 1.0     # [m] Lookahead distance
Kp = 12.2     # Speed proportional gain
WB = 0.2      # [m] Wheelbase of the robot

class PurePursuitROS(Node):

    def __init__(self):
        super().__init__('pure_pursuit_ros')
        self.create_subscription(ModelStates, '/gazebo/model_states', self.gazebo_callback, 10)
        self.waypoints = self.load_path("path.yaml")

        self.pub_steering = self.create_publisher(
            Float64MultiArray,
            "/position_controllers/commands",
            10
        )
        self.pub_wheel_spd = self.create_publisher(
            Float64MultiArray, 
            '/velocity_controllers/commands', 
            10
        )
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.target_speed = 0.5  
        self.old_nearest_point_index = None

        # Setup real-time plotting
        self.robot_x = []
        self.robot_y = []
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 6))

    def load_path(self, filename):
        if not os.path.isabs(filename):
            ros_workspace = os.getenv("ROS_WORKSPACE")
            if ros_workspace is None:
                script_dir = os.path.dirname(os.path.realpath(__file__))
                ros_workspace = script_dir.split('/src/')[0]
            filename = os.path.join(ros_workspace, "src", "FRA532_LAB1_6702_6703", "path_tracking", "data", filename)
        
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
        return np.array([(point['x'], point['y']) for point in data['path']])

    def search_target_index(self):
        """Find the nearest waypoint and determine the lookahead index."""
        if self.old_nearest_point_index is None:
            # Find the closest waypoint
            distances = np.hypot(self.waypoints[:, 0] - self.x, self.waypoints[:, 1] - self.y)
            ind = int(np.argmin(distances))
        else:
            ind = self.old_nearest_point_index
            while ind < len(self.waypoints) - 1:
                distance_next = np.hypot(self.waypoints[ind+1, 0] - self.x, self.waypoints[ind+1, 1] - self.y)
                current_distance = np.hypot(self.waypoints[ind, 0] - self.x, self.waypoints[ind, 1] - self.y)
                if distance_next > current_distance:
                    break
                ind += 1

        self.old_nearest_point_index = ind

        # Calculate lookahead distance
        Lf = k * self.v + Lfc
        while Lf > np.hypot(self.waypoints[ind, 0] - self.x, self.waypoints[ind, 1] - self.y):
            if (ind + 1) >= len(self.waypoints):
                break
            ind += 1

        return ind, Lf

    def pure_pursuit_control(self):
        """Compute the required steering angle."""
        ind, Lf = self.search_target_index()
        if ind < len(self.waypoints):
            tx, ty = self.waypoints[ind]
        else:
            tx, ty = self.waypoints[-1]
            ind = len(self.waypoints) - 1

        # Compute angle to target
        alpha = math.atan2(ty - self.y, tx - self.x) - self.yaw
        alpha = (alpha + np.pi) % (2 * np.pi) - np.pi  # Normalize angle

        # Sine-based steering control
        delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)
        return delta, ind

    def proportional_control(self):
        """Compute acceleration using proportional speed control."""
        return Kp * (self.target_speed - self.v)

    def quaternion_to_euler(self, q):
        """Convert quaternion to Euler yaw."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def gazebo_callback(self, msg):
        """Receive real-time pose data from Gazebo and apply Pure Pursuit control."""
        try:
            index = msg.name.index("limo")  # เปลี่ยนเป็นชื่อ robot ของคุณ
        except ValueError:
            self.get_logger().error("❌ Robot model not found in Gazebo!")
            return

        pose = msg.pose[index]
        self.x, self.y = pose.position.x, pose.position.y
        self.yaw = self.quaternion_to_euler(pose.orientation)

        # ตั้ง target speed ที่ 5.5 m/s
        self.target_speed = 5.5  

        # คำนวณ Pure Pursuit steering
        steering_cmd, target_ind = self.pure_pursuit_control()

        # Speed Control: คำนวณ acceleration ด้วย gain ที่เพิ่มขึ้น
        acceleration = 3.0 * (self.target_speed - self.v)
        self.v += acceleration * 0.1  # อัปเดตความเร็ว
        self.v = min(self.v, 15.5)      # จำกัดความเร็วสูงสุด

        # ส่งคำสั่งควบคุม
        self.publish_steering(steering_cmd)
        self.publish_wheel_speed(self.v)

        self.get_logger().info(
            f"📍 Pose: x={self.x:.3f}, y={self.y:.3f}, yaw={math.degrees(self.yaw):.2f}° | "
            f"🔄 Steering: {math.degrees(steering_cmd):.2f}° | 🚀 Speed: {self.v:.2f} m/s"
        )

        # เก็บข้อมูลสำหรับ plotting
        self.robot_x.append(self.x)
        self.robot_y.append(self.y)
        self.update_plot()

    def publish_steering(self, steering):
        """Publish steering commands to Gazebo."""
        steering_msg = Float64MultiArray()
        # สมมุติว่า steering command เดียวกันส่งให้ทั้งสองล้อหน้าครับ
        steering_msg.data = [steering, steering]
        self.pub_steering.publish(steering_msg)

    def publish_wheel_speed(self, speed):
        """Publish speed commands to Gazebo."""
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [speed, speed]
        self.pub_wheel_spd.publish(wheel_msg)

    def update_plot(self):
        # """Update real-time plot."""
        # self.ax.clear()
        # self.ax.plot(self.waypoints[:, 0], self.waypoints[:, 1], 'go-', label="Planned Path")
        # self.ax.plot(self.robot_x, self.robot_y, 'r.-', label="Actual Path")
        # self.ax.scatter(self.robot_x[-1], self.robot_y[-1], c='purple', marker='x', label="Current Position")
        # self.ax.set_title("Pure Pursuit Controller")
        # self.ax.legend()
        # plt.draw()
        # plt.pause(0.001)
        pass

def main():
    rclpy.init()
    node = PurePursuitROS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

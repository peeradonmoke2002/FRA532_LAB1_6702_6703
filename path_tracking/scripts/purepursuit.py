#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
import yaml
import numpy as np
import math
import time
import matplotlib.pyplot as plt
import rospkg
# ✅ Pure Pursuit Parameters
k = 0.1  # Look forward gain
Lfc = 1.0  # [m] Lookahead distance
Kp = 12.2  # Speed proportional gain
WB = 0.2  # [m] Wheelbase of the robot

class PurePursuitROS(Node):

    def __init__(self):
        super().__init__('pure_pursuit_ros')
        self.create_subscription(ModelStates, '/gazebo/model_states', self.gazebo_callback, 10)
        self.waypoints = self.load_path('/home/tang/ros2_lab1_m/src/FRA532_LAB1_6702_6703/path_tracking/data/path.yaml')
        self.pub_steering = self.create_publisher(JointTrajectory, '/joint_trajectory_position_controller/joint_trajectory', 10)
        self.pub_wheel_spd = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)
        
        #  Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.target_speed = 0.5  
        self.old_nearest_point_index = None

        #  Setup real-time plotting
        self.robot_x = []
        self.robot_y = []
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 6))

    def load_path(self, filename):
        """ Load waypoints from YAML file """
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
        if 'path' not in data:
            self.get_logger().error(f"⚠️ Key 'path' not found in {filename}")
            return []
        return np.array([(point['x'], point['y']) for point in data['path']])

    def search_target_index(self):
        """ Find the nearest waypoint and determine the lookahead index """
        if self.old_nearest_point_index is None:
            # ✅ Find the closest waypoint
            distances = np.hypot(self.waypoints[:, 0] - self.x, self.waypoints[:, 1] - self.y)
            ind = np.argmin(distances)
        else:
            ind = self.old_nearest_point_index
            while ind < len(self.waypoints) - 1:
                distance_next = np.hypot(self.waypoints[ind+1, 0] - self.x, self.waypoints[ind+1, 1] - self.y)
                if distance_next > np.hypot(self.waypoints[ind, 0] - self.x, self.waypoints[ind, 1] - self.y):
                    break
                ind += 1

        self.old_nearest_point_index = ind

        #  Calculate lookahead distance
        Lf = k * self.v + Lfc
        while Lf > np.hypot(self.waypoints[ind, 0] - self.x, self.waypoints[ind, 1] - self.y):
            if (ind + 1) >= len(self.waypoints):
                break
            ind += 1

        return ind, Lf

    def pure_pursuit_control(self):
        """ Compute the required steering angle """
        ind, Lf = self.search_target_index()

        if ind < len(self.waypoints):
            tx, ty = self.waypoints[ind]
        else:
            tx, ty = self.waypoints[-1]
            ind = len(self.waypoints) - 1

        # ✅ Compute angle to target
        alpha = math.atan2(ty - self.y, tx - self.x) - self.yaw
        alpha = (alpha + np.pi) % (2 * np.pi) - np.pi  # Normalize angle

        # ✅ Compute steering angle
        delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)
        return delta, ind

    def proportional_control(self):
        """ Compute acceleration using proportional speed control """
        return Kp * (self.target_speed - self.v)

    def quaternion_to_euler(self, q):
        """ Convert quaternion to Euler yaw """
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def gazebo_callback(self, msg):
        """ Receive real-time pose data from Gazebo and apply Pure Pursuit control """
        try:
            index = msg.name.index("limo")  # Change to match your robot's model name
        except ValueError:
            self.get_logger().error("❌ Robot model not found in Gazebo!")
            return

        pose = msg.pose[index]
        self.x, self.y = pose.position.x, pose.position.y
        self.yaw = self.quaternion_to_euler(pose.orientation)

        # Set target speed to 5.5 m/s
        self.target_speed = 5.5  

        # Compute Pure Pursuit steering
        steering_cmd, target_ind = self.pure_pursuit_control()

        # Speed Control: Increase acceleration gain (Kp)
        acceleration = 3.0 * (self.target_speed - self.v)  # Increased Kp for faster acceleration
        self.v += acceleration * 0.1  # Faster speed update

        # Ensure speed is limited to prevent instability
        self.v = min(self.v, 15.5)  # ✅ Max speed set to 15.5 m/s

        # Publish control commands
        self.publish_steering(steering_cmd)
        self.publish_wheel_speed(self.v)

        self.get_logger().info(f"📍 Pose: x={self.x:.3f}, y={self.y:.3f}, yaw={math.degrees(self.yaw):.2f}° | 🔄 Steering: {math.degrees(steering_cmd):.2f}° | 🚀 Speed: {self.v:.2f} m/s")

        # Store data for plotting
        self.robot_x.append(self.x)
        self.robot_y.append(self.y)

        # Update real-time plot
        self.update_plot()


    def publish_steering(self, steering):
        """ Publish steering commands to Gazebo """
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['front_left_steering', 'front_right_steering']
        point = JointTrajectoryPoint()
        point.positions = [steering, steering]
        traj_msg.points.append(point)
        self.pub_steering.publish(traj_msg)

    def publish_wheel_speed(self, speed):
        """ Publish speed commands to Gazebo """
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [speed, speed]
        self.pub_wheel_spd.publish(wheel_msg)

    def update_plot(self):
        # """ Update real-time plot """
        # self.ax.clear()
        # self.ax.plot(self.waypoints[:, 0], self.waypoints[:, 1], 'go-', label="Planned Path")
        # self.ax.plot(self.robot_x, self.robot_y, 'r.-', label="Actual Path")
        # self.ax.scatter(self.robot_x[-1], self.robot_y[-1], c='purple', marker='x', label="Current Position")
        # self.ax.set_title("Pure Pursuit Controller")
        # self.ax.legend()
        # plt.draw()
        # plt.pause(0.1)
        pass

def main():
    rclpy.init()
    node = PurePursuitROS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

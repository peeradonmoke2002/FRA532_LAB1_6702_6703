#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
import yaml
import numpy as np
import cvxpy as cp
import math
import matplotlib.pyplot as plt

# ✅ MPC Parameters
HORIZON = 10  # Prediction horizon
DT = 0.1  # Discretization timestep
WB = 0.2  # Wheelbase of the robot
TARGET_SPEED = 5.5  # m/s
MAX_STEERING = np.radians(30)  # Max steering angle

class MPCController(Node):

    def __init__(self):
        super().__init__('mpc_controller')

        # ✅ Subscribe to Gazebo model states
        self.create_subscription(ModelStates, '/gazebo/model_states', self.gazebo_callback, 10)

        # ✅ Load waypoints from path.yaml
        self.waypoints = self.load_path('/home/tang/ros2_lab1_m/src/FRA532_LAB1_6702_6703/robot_controller/data/path.yaml')

        # ✅ ROS2 Publishers
        self.pub_steering = self.create_publisher(JointTrajectory, '/joint_trajectory_position_controller/joint_trajectory', 10)
        self.pub_wheel_spd = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)

        # ✅ Robot state
        self.x, self.y, self.yaw, self.v = 0.0, 0.0, 0.0, 0.0
        self.prev_steering = 0.0  # Store last steering command
        self.robot_x, self.robot_y = [], []  # Store trajectory for plotting

        # ✅ Setup real-time plotting
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

    def find_nearest_waypoint(self):
        """ Find the nearest waypoint to the robot """
        distances = np.hypot(self.waypoints[:, 0] - self.x, self.waypoints[:, 1] - self.y)
        return np.argmin(distances)

    def linear_mpc_control(self):
        """ Compute the optimal control input using Linear MPC """

        # ✅ Define MPC Variables
        x = cp.Variable(HORIZON + 1)  # X positions
        y = cp.Variable(HORIZON + 1)  # Y positions
        yaw = cp.Variable(HORIZON + 1)  # Yaw angles
        v = cp.Variable(HORIZON + 1)  # Speeds
        delta = cp.Variable(HORIZON)  # Steering inputs

        # ✅ Precompute cos(yaw) and sin(yaw)
        yaw_values = np.linspace(self.yaw, self.yaw + (HORIZON * DT) * (self.v / WB) * self.prev_steering, HORIZON)
        cos_yaw = np.cos(yaw_values)
        sin_yaw = np.sin(yaw_values)

        # ✅ Define Constraints & Cost Function
        constraints = []
        cost = 0.0

        # ✅ Set Initial State Constraints
        constraints += [
            x[0] == self.x,
            y[0] == self.y,
            yaw[0] == self.yaw,
            v[0] == self.v
        ]

        nearest_index = self.find_nearest_waypoint()

        # ✅ Loop over prediction horizon
        for t in range(HORIZON):
            # ✅ Vehicle model constraints (Precomputed cos/sin)
            constraints += [
                x[t + 1] == x[t] + cp.multiply(v[t], cos_yaw[t]) * DT,  # ✅ FIXED
                y[t + 1] == y[t] + cp.multiply(v[t], sin_yaw[t]) * DT,  # ✅ FIXED
                yaw[t + 1] == yaw[t] + cp.multiply((v[t] / WB), delta[t]) * DT,  # ✅ FIXED (NO TAN)
                v[t + 1] == TARGET_SPEED  # Fix speed at desired target
            ]

            # ✅ Steering Constraints
            constraints += [
                cp.abs(delta[t]) <= MAX_STEERING  # Limit steering angle
            ]

            # ✅ Cost Function (Minimize Deviation from Path)
            cost += cp.square(x[t + 1] - self.waypoints[nearest_index, 0])
            cost += cp.square(y[t + 1] - self.waypoints[nearest_index, 1])
            cost += cp.square(delta[t] - self.prev_steering)  # Smooth steering

        # ✅ Solve Optimization Problem
        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.OSQP)  # Use Quadratic Programming solver

        # ✅ Apply Optimal Steering Command
        optimal_steering = delta.value[0] if delta.value is not None else 0.0
        return np.clip(optimal_steering, -MAX_STEERING, MAX_STEERING)



    def quaternion_to_euler(self, q):
        """ Convert quaternion to Euler yaw """
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def gazebo_callback(self, msg):
        """ Receive real-time pose data from Gazebo and apply MPC control """
        try:
            index = msg.name.index("limo")  # ✅ Change to match your robot's model name
        except ValueError:
            self.get_logger().error("❌ Robot model not found in Gazebo!")
            return

        pose = msg.pose[index]
        self.x, self.y = pose.position.x, pose.position.y
        self.yaw = self.quaternion_to_euler(pose.orientation)

        # ✅ Compute MPC steering control
        steering_cmd = self.linear_mpc_control()
        self.prev_steering = steering_cmd  # Store for next iteration

        # ✅ Publish control commands
        self.publish_steering(steering_cmd)
        self.publish_wheel_speed(TARGET_SPEED)

        self.get_logger().info(f"📍 Pose: x={self.x:.3f}, y={self.y:.3f}, yaw={math.degrees(self.yaw):.2f}° | 🔄 Steering: {math.degrees(steering_cmd):.2f}° | 🚀 Speed: {TARGET_SPEED:.2f} m/s")

        # ✅ Store data for plotting
        self.robot_x.append(self.x)
        self.robot_y.append(self.y)

        # ✅ Update real-time plot
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
        """ Update real-time plot """
        self.ax.clear()
        self.ax.plot(self.waypoints[:, 0], self.waypoints[:, 1], 'go-', label="Planned Path")
        self.ax.plot(self.robot_x, self.robot_y, 'r.-', label="Actual Path")
        self.ax.scatter(self.robot_x[-1], self.robot_y[-1], c='purple', marker='x', label="Current Position")
        self.ax.legend()
        plt.draw()
        plt.pause(0.1)

def main():
    rclpy.init()
    node = MPCController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

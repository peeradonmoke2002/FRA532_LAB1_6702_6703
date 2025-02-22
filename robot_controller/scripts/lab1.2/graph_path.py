#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
import yaml
import numpy as np
import matplotlib.pyplot as plt
import time

class RealtimePathPlot(Node):
    def __init__(self):
        super().__init__('realtime_path_plotter')

        # ✅ Subscribe to Gazebo model states
        self.create_subscription(ModelStates, '/gazebo/model_states', self.gazebo_callback, 10)

        # ✅ Load waypoints from `path.yaml`
        self.waypoints = self.load_path('/home/tang/ros2_lab1_m/src/FRA532_LAB1_6702_6703/robot_controller/data/path.yaml')

        # ✅ Store real-time robot position
        self.robot_x = []
        self.robot_y = []

        # ✅ Setup Matplotlib
        plt.ion()  # Enable interactive mode for real-time updates
        self.fig, self.ax = plt.subplots(figsize=(8, 6))

    def load_path(self, filename):
        """ Load waypoints from YAML file """
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
        if 'path' not in data:
            self.get_logger().error(f"⚠️ Key 'path' not found in {filename}")
            return []
        return np.array([(point['x'], point['y']) for point in data['path']])

    def gazebo_callback(self, msg):
        """ Receive real-time pose data from Gazebo and update trajectory """
        try:
            index = msg.name.index("limo")  # ✅ Change "limo" to match your robot model name
        except ValueError:
            self.get_logger().error("❌ Robot model not found in Gazebo!")
            return

        pose = msg.pose[index]
        x, y = pose.position.x, pose.position.y

        # ✅ Store real-time position
        self.robot_x.append(x)
        self.robot_y.append(y)

        self.get_logger().info(f"📍 Gazebo Pose: x={x:.3f}, y={y:.3f}")

        # ✅ Update real-time plot
        self.update_plot()

    def update_plot(self):
        """ Update the real-time plot of X-Y position """
        self.ax.clear()
        self.ax.set_xlabel("X Position (m)")
        self.ax.set_ylabel("Y Position (m)")
        self.ax.set_title("MPC control")
        self.ax.grid()

        # ✅ Plot planned path (Green line)
        if len(self.waypoints) > 0:
            self.ax.plot(self.waypoints[:, 0], self.waypoints[:, 1], 'go-', label="Planned Path (path.yaml)")

        # ✅ Plot actual trajectory from Gazebo (Red line)
        if len(self.robot_x) > 0:
            self.ax.plot(self.robot_x, self.robot_y, 'r.-', label="Actual Path (Gazebo)")

        # ✅ Mark Start and Current Position
        if len(self.robot_x) > 0:
            self.ax.scatter(self.robot_x[0], self.robot_y[0], c='blue', marker='o', label="Start Position")
            self.ax.scatter(self.robot_x[-1], self.robot_y[-1], c='purple', marker='x', label="Current Position")

        self.ax.legend()
        plt.draw()
        plt.pause(0.1)  # ✅ Ensure real-time updates

def main(args=None):
    rclpy.init(args=args)
    node = RealtimePathPlot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

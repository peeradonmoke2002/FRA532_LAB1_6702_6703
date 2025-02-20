#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
import numpy as np
import matplotlib.pyplot as plt
import time

class SpeedPlot(Node):
    def __init__(self):
        super().__init__('speed_plotter')

        # ✅ Subscribe to Gazebo model states
        self.create_subscription(ModelStates, '/gazebo/model_states', self.gazebo_callback, 10)

        # ✅ Store speed and timestamps
        self.speeds = []
        self.timestamps = []

        # ✅ Exponential Moving Average (EMA) filter parameters
        self.alpha = 0.05  # Smoothing factor (0.0 = no filtering, 1.0 = no smoothing)
        self.filtered_speed = 0.0  # Initial speed estimate

        # ✅ Setup Matplotlib for real-time plotting
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.start_time = time.time()

    def gazebo_callback(self, msg):
        """ Receive real-time pose data from Gazebo and compute speed """
        try:
            index = msg.name.index("limo")  # ✅ Change "limo" to match your robot model name
        except ValueError:
            self.get_logger().error("❌ Robot model not found in Gazebo!")
            return

        # ✅ Extract velocity from Gazebo message
        velocity = msg.twist[index].linear
        speed = np.hypot(velocity.x, velocity.y)  # Compute magnitude of velocity vector

        # ✅ Apply Exponential Moving Average (EMA) Filter
        self.filtered_speed = self.alpha * speed + (1 - self.alpha) * self.filtered_speed

        # ✅ Store filtered speed and timestamp
        current_time = time.time() - self.start_time
        self.speeds.append(self.filtered_speed)
        self.timestamps.append(current_time)

        self.get_logger().info(f"📊 Raw Speed: {speed:.3f} m/s | Filtered Speed: {self.filtered_speed:.3f} m/s at {current_time:.2f} sec")

        # ✅ Update real-time plot
        self.update_plot()

    def update_plot(self):
        """ Update the real-time Speed-Time plot """
        self.ax.clear()
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Speed (m/s)")
        self.ax.set_title("Real-Time Speed Plot")
        self.ax.grid()

        if len(self.speeds) > 1:
            self.ax.plot(self.timestamps, self.speeds, 'r-', label=" Speed (m/s)")
            self.ax.legend()

        plt.draw()
        plt.pause(0.1)  # ✅ Ensure real-time updates

def main(args=None):
    rclpy.init(args=args)
    node = SpeedPlot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

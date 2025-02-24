#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np

class GPSPlotter(Node):
    def __init__(self):
        super().__init__('gps_plotter')
        # Subscribe to the noisy odometry topic published by your GPS emulator node.
        self.subscription = self.create_subscription(
            Odometry,
            '/odom_noisy',
            self.odom_callback,
            10
        )
        # Lists to store x and y positions.
        self.x_data = []
        self.y_data = []
        
        # Set up Matplotlib in interactive mode.
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_title('Noisy Odometry Data (Dynamic Focus)')
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        
        # Create an empty scatter plot with green dots.
        self.scatter = self.ax.scatter([], [], c='g', marker='.', s=10)
        
        # Timer callback to update the plot every 0.1 seconds.
        self.timer = self.create_timer(0.1, self.update_plot)

    def odom_callback(self, msg: Odometry):
        # Append new x and y positions from the Odometry message.
        self.x_data.append(msg.pose.pose.position.x)
        self.y_data.append(msg.pose.pose.position.y)

    def update_plot(self):
        if self.x_data and self.y_data:
            offsets = np.column_stack((self.x_data, self.y_data))
            self.scatter.set_offsets(offsets)
            
            # Center on the latest data point.
            last_x = self.x_data[-1]
            last_y = self.y_data[-1]
            
            # Compute overall extents of data.
            x_min = min(self.x_data)
            x_max = max(self.x_data)
            y_min = min(self.y_data)
            y_max = max(self.y_data)
            
            # Define a minimum margin.
            min_margin = 5.0
            
            # Use the larger of the minimum margin or half the range of data.
            x_margin = max(min_margin, (x_max - x_min) / 2)
            y_margin = max(min_margin, (y_max - y_min) / 2)
            
            # Set the limits centered around the latest point.
            self.ax.set_xlim(last_x - x_margin, last_x + x_margin)
            self.ax.set_ylim(last_y - y_margin, last_y + y_margin)
            
        plt.draw()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    gps_plotter = GPSPlotter()
    try:
        rclpy.spin(gps_plotter)
    except KeyboardInterrupt:
        pass
    gps_plotter.destroy_node()
    rclpy.shutdown()
    # Turn off interactive mode and display the final plot.
    plt.ioff()
    plt.show()

if __name__ == '__main__':
    main()

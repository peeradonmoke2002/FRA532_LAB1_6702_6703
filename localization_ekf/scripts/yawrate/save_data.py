#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import csv

class EKFVisualization(Node):
    def __init__(self):
        super().__init__('ekf_visualization')

        # Subscribe to odometry topics
        self.sub_dead_reckoning = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_gps = self.create_subscription(Odometry, '/odom_noisy', self.gps_callback, 10)
        self.sub_ekf = self.create_subscription(Odometry, '/odometry/filtered', self.ekf_callback, 10)

        # Store trajectory data
        self.true_traj = []  # Blue: True trajectory (if available)
        self.dr_traj = []    # Black: Dead reckoning
        self.gps_obs = []    # Green: GPS observations
        self.ekf_traj = []   # Red: EKF estimated trajectory

        # Open CSV files in write mode and create CSV writer objects.
        # Files will be saved in the directory where you launch this node.
        self.f_dr = open("dead_reckoning.csv", "w", newline="")
        self.f_gps = open("gps_obs.csv", "w", newline="")
        self.f_ekf = open("ekf_estimated.csv", "w", newline="")
        self.writer_dr = csv.writer(self.f_dr)
        self.writer_gps = csv.writer(self.f_gps)
        self.writer_ekf = csv.writer(self.f_ekf)

        # Write headers to the CSV files.
        self.writer_dr.writerow(["x", "y"])
        self.writer_gps.writerow(["x", "y"])
        self.writer_ekf.writerow(["x", "y"])

        # Initialize live plotting
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.timer = self.create_timer(0.1, self.plot_trajectory)  # Update every 100ms

    def odom_callback(self, msg):
        """Store dead reckoning trajectory from odometry and save immediately."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.dr_traj.append((x, y))
        self.writer_dr.writerow([x, y])
        self.f_dr.flush()  # Flush to write to disk immediately

    def gps_callback(self, msg):
        """Store noisy GPS observations and save immediately."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.gps_obs.append((x, y))
        self.writer_gps.writerow([x, y])
        self.f_gps.flush()

    def ekf_callback(self, msg):
        """Store EKF estimated trajectory and save immediately."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.ekf_traj.append((x, y))
        self.writer_ekf.writerow([x, y])
        self.f_ekf.flush()

    def plot_trajectory(self):
        """Real-time plotting of the EKF localization results."""
        self.ax.clear()

        # Plot Dead Reckoning Trajectory (Black dashed line)
        if len(self.dr_traj) > 1:
            dr_x, dr_y = zip(*self.dr_traj)
            self.ax.plot(dr_x, dr_y, 'k--', label="Dead Reckoning")

        # Plot GPS Observations (Green dots)
        if len(self.gps_obs) > 0:
            gps_x, gps_y = zip(*self.gps_obs)
            self.ax.scatter(gps_x, gps_y, color='g', marker='o', label="GPS Observations")

        # Plot EKF Estimated Trajectory (Red line)
        if len(self.ekf_traj) > 1:
            ekf_x, ekf_y = zip(*self.ekf_traj)
            self.ax.plot(ekf_x, ekf_y, 'r-', label="EKF Estimated Trajectory")

        self.ax.set_xlabel("X Position")
        self.ax.set_ylabel("Y Position")
        self.ax.set_title("Sensor Fusion Localization with EKF")
        self.ax.legend()
        self.ax.grid(True)

        plt.pause(0.01)  # Pause for live update

    def destroy_node(self):
        # Close file handles when the node is destroyed.
        self.f_dr.close()
        self.f_gps.close()
        self.f_ekf.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EKFVisualization()
    try:
        plt.ion()  # Enable interactive mode for live plotting
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down EKF visualization node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

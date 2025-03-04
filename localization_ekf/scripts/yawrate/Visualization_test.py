#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry

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
        self.covariances = []  # (No longer used for plotting ellipses)

        # Initialize live plotting
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.timer = self.create_timer(0.1, self.plot_trajectory)  # Update every 100ms

    def odom_callback(self, msg):
        """Store dead reckoning trajectory from odometry."""
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.dr_traj.append((x, y))

    def gps_callback(self, msg):
        """Store noisy GPS observations."""
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.gps_obs.append((x, y))

    def ekf_callback(self, msg):
        """Store EKF estimated trajectory."""
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.ekf_traj.append((x, y))
        # Optionally, if you want to store covariances, you can keep this:
        # cov_matrix = np.array(msg.pose.covariance).reshape(6, 6)[:2, :2]
        # self.covariances.append(cov_matrix)

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

        # (Covariance ellipses removed.)

        # Configure plot
        self.ax.set_xlabel("X Position")
        self.ax.set_ylabel("Y Position")
        self.ax.set_title("Sensor Fusion Localization with EKF")
        self.ax.legend()
        self.ax.grid(True)

        plt.pause(0.01)  # Pause for live update

def main(args=None):
    rclpy.init(args=args)
    node = EKFVisualization()
    try:
        plt.ion()  # Enable interactive mode for real-time plotting
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down EKF visualization node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
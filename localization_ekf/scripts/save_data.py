#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Twist, Vector3
import matplotlib.pyplot as plt
import numpy as np
import csv
import math

class EKFVisualization(Node):
    def __init__(self):
        super().__init__('ekf_visualization')

        # Subscribers for trajectories:
        self.sub_dead_reckoning = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_ekf = self.create_subscription(Odometry, '/odometry/filtered', self.ekf_callback, 10)
        # Subscribe to GPS observations as PoseStamped
        self.sub_gps = self.create_subscription(PoseStamped, '/gps', self.gps_callback, 10)
        # Subscribe to JointState for steering angles and rear wheel angles (for slip detection)
        self.sub_wheel = self.create_subscription(JointState, '/joint_states', self.wheel_callback, 10)

        # Store trajectory data
        self.dr_traj = []       # Dead reckoning (Black dashed)
        self.gps_obs = []       # GPS observations (Green dots)
        self.ekf_traj = []      # EKF estimated trajectory (Red line)
        self.front_wheel_angles = []  # (front_left, front_right)
        self.rear_wheel_angles = []   # (rear_left, rear_right)
        self.rear_slip = []     # Slip metric computed from rear wheels (% difference)

        # Open CSV files for logging trajectories and wheel angles
        self.f_dr = open("dead_reckoning.csv", "w", newline="")
        self.f_gps = open("gps_obs.csv", "w", newline="")
        self.f_ekf = open("ekf_estimated.csv", "w", newline="")
        self.f_front = open("front_wheel_angles.csv", "w", newline="")
        self.f_rear = open("rear_wheel_angles.csv", "w", newline="")
        self.f_slip = open("rear_slip.csv", "w", newline="")

        self.writer_dr = csv.writer(self.f_dr)
        self.writer_gps = csv.writer(self.f_gps)
        self.writer_ekf = csv.writer(self.f_ekf)
        self.writer_front = csv.writer(self.f_front)
        self.writer_rear = csv.writer(self.f_rear)
        self.writer_slip = csv.writer(self.f_slip)

        self.writer_dr.writerow(["x", "y"])
        self.writer_gps.writerow(["x", "y"])
        self.writer_ekf.writerow(["x", "y"])
        self.writer_front.writerow(["front_left_steering", "front_right_steering"])
        self.writer_rear.writerow(["rear_left_wheel", "rear_right_wheel"])
        self.writer_slip.writerow(["slip_percentage"])

        # Initialize live plotting with 2 subplots: trajectories and wheel angles/slip
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))
        self.timer = self.create_timer(0.1, self.plot_trajectory)  # Update every 100ms

    def odom_callback(self, msg):
        """Store dead reckoning trajectory from odometry and log to CSV."""
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.dr_traj.append((x, y))
        self.writer_dr.writerow([x, y])
        self.f_dr.flush()

    def gps_callback(self, msg):
        """Store GPS observations from PoseStamped and log to CSV."""
        x, y = msg.pose.position.x, msg.pose.position.y
        self.gps_obs.append((x, y))
        self.writer_gps.writerow([x, y])
        self.f_gps.flush()

    def ekf_callback(self, msg):
        """Store EKF estimated trajectory and log to CSV."""
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.ekf_traj.append((x, y))
        self.writer_ekf.writerow([x, y])
        self.f_ekf.flush()

    def wheel_callback(self, msg: JointState):
        """
        Extract and store front steering angles and rear wheel angles from the JointState message.
        We assume joint names:
            Front: "front_left_steering", "front_right_steering"
            Rear:  "rear_left_wheel", "rear_right_wheel"
        """
        front_left = None
        front_right = None
        rear_left = None
        rear_right = None

        for name, pos in zip(msg.name, msg.position):
            if name == "front_left_steering":
                front_left = pos
            elif name == "front_right_steering":
                front_right = pos
            elif name == "rear_left_wheel":
                rear_left = pos
            elif name == "rear_right_wheel":
                rear_right = pos
        
        if front_left is not None and front_right is not None:
            self.front_wheel_angles.append((front_left, front_right))
            self.writer_front.writerow([front_left, front_right])
            self.f_front.flush()

        if rear_left is not None and rear_right is not None:
            self.rear_wheel_angles.append((rear_left, rear_right))
            self.writer_rear.writerow([rear_left, rear_right])
            self.f_rear.flush()
            # Compute slip metric for rear wheels:
            avg_rear = (abs(rear_left) + abs(rear_right)) / 2.0
            slip = (abs(rear_left - rear_right) / avg_rear * 100) if avg_rear != 0 else 0
            self.rear_slip.append(slip)
            self.writer_slip.writerow([slip])
            self.f_slip.flush()
            self.get_logger().debug(f"Rear wheel angles: Left={rear_left:.3f}, Right={rear_right:.3f}, Slip={slip:.2f}%")

    def plot_trajectory(self):
        """Real-time plotting of the localization results and wheel angles/slip."""
        # Clear subplots
        self.ax1.clear()
        self.ax2.clear()

        # Subplot 1: Plot trajectories
        if len(self.dr_traj) > 1:
            dr_x, dr_y = zip(*self.dr_traj)
            self.ax1.plot(dr_x, dr_y, 'k--', label="Dead Reckoning")
        if len(self.gps_obs) > 0:
            gps_x, gps_y = zip(*self.gps_obs)
            self.ax1.scatter(gps_x, gps_y, color='g', marker='o', label="GPS Observations")
        if len(self.ekf_traj) > 1:
            ekf_x, ekf_y = zip(*self.ekf_traj)
            self.ax1.plot(ekf_x, ekf_y, 'r-', label="EKF Estimated Trajectory")
        self.ax1.set_xlabel("X Position")
        self.ax1.set_ylabel("Y Position")
        self.ax1.set_title("Trajectories")
        self.ax1.legend()
        self.ax1.grid(True)

        # Subplot 2: Plot front wheel angles and rear slip metric
        # Front wheel angles
        if len(self.front_wheel_angles) > 0:
            indices = np.arange(len(self.front_wheel_angles))
            front_left_angles = [angle[0] for angle in self.front_wheel_angles]
            front_right_angles = [angle[1] for angle in self.front_wheel_angles]
            self.ax2.plot(indices, front_left_angles, 'b-', label="Front Left Steering")
            self.ax2.plot(indices, front_right_angles, 'm-', label="Front Right Steering")
        # Rear slip metric in red
        if len(self.rear_slip) > 0:
            indices_slip = np.arange(len(self.rear_slip))
            self.ax2.plot(indices_slip, self.rear_slip, 'r-', label="Rear Wheel Slip (%)")

        self.ax2.set_xlabel("Measurement Index")
        self.ax2.set_ylabel("Angle (rad) / Slip (%)")
        self.ax2.set_title("Steering Angles and Rear Wheel Slip")
        self.ax2.legend()
        self.ax2.grid(True)

        plt.pause(0.01)

    def destroy_node(self):
        # Close CSV files upon shutdown.
        self.f_dr.close()
        self.f_gps.close()
        self.f_ekf.close()
        self.f_front.close()
        self.f_rear.close()
        self.f_slip.close()
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
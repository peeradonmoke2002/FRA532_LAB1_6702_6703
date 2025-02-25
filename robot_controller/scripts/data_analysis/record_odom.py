#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import csv
import time
import atexit
import os

class OdomRecorderNode(Node):

    def __init__(self):
        super().__init__('odom_recorder')
        queue_size = 10

        # Subscribe to the estimated odometry topic
        self.create_subscription(Odometry, '/odom', self.odom_callback, queue_size)

        # Subscribe to the Gazebo model states topic
        self.create_subscription(ModelStates, '/gazebo/model_states', self.model_states_callback, queue_size)

        odom_path = "/home/peeradon/FRA532_MobileRobot/src/"
        # Open CSV file for recording odometry estimates
        self.odom_file_name = 'odom_estimated.csv'
        self.odom_csv_file = open(os.path.join(odom_path, self.odom_file_name), 'w', newline='')
        self.odom_csv_writer = csv.writer(self.odom_csv_file)
        self.odom_csv_writer.writerow(['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])  # Numeric timestamp format

        # Open CSV file for recording Gazebo ground truth (for model "limo")
        self.gt_file_name = 'odom_gazebo_limo.csv'
        self.gt_csv_file = open(os.path.join(odom_path, self.gt_file_name), 'w', newline='')
        self.gt_csv_writer = csv.writer(self.gt_csv_file)
        self.gt_csv_writer.writerow(['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

        # Ensure files are closed when the script exits
        atexit.register(self.cleanup)

    def get_timestamp(self):
        """
        Returns the current time as a **UNIX timestamp** (seconds since epoch).
        """
        return time.time()

    def odom_callback(self, msg: Odometry):
        """
        Callback for the estimated odometry. Records timestamp and (x, y, z) position.
        """
        timestamp = self.get_timestamp()
        x, y, z = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
        qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        self.get_logger().info(f"[ODOM] Position: ({x}, {y}, {z})")
        
        self.odom_csv_writer.writerow([timestamp, x, y, z, qx, qy, qz, qw])
        self.odom_csv_file.flush()  # Ensure data is written immediately

    def model_states_callback(self, msg: ModelStates):
        """
        Callback for Gazebo model states. Looks for a model named "limo" and records its pose.
        """
        if "limo" in msg.name:
            index = msg.name.index("limo")
            pose = msg.pose[index]
            timestamp = self.get_timestamp()

            x, y, z = pose.position.x, pose.position.y, pose.position.z
            qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
            
            self.get_logger().info(f"[GAZEBO] 'limo' Position: ({x}, {y}, {z})")
            
            self.gt_csv_writer.writerow([timestamp, x, y, z, qx, qy, qz, qw])
            self.gt_csv_file.flush()  # Ensure data is written immediately
        else:
            self.get_logger().warn("Model 'limo' not found in ModelStates.")

    def cleanup(self):
        """Ensures files are closed properly when the script exits."""
        self.odom_csv_file.close()
        self.gt_csv_file.close()
        self.get_logger().info("CSV files closed successfully.")

def main(args=None):
    rclpy.init(args=args)
    node = OdomRecorderNode()
    rclpy.spin(node)
    node.cleanup()  # Ensure cleanup before shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import csv
import time
import os
import atexit

class OdomRecorderNode(Node):
    def __init__(self):
        super().__init__('odom_recorder')
        # Define the output directory and ensure it exists.
        self.output_dir = os.path.expanduser("~/FRA532_MobileRobot_LAB1_6702_6703/src/FRA532_LAB1_6702_6703/robot_controller/scripts/data_analysis/record_data")
        os.makedirs(self.output_dir, exist_ok=True)
        # Open a single CSV file to record all messages.
        csv_file_path = os.path.join(self.output_dir, 'odometry_data.csv')
        self.csv_file = open(csv_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # Write header with a source field.
        self.csv_writer.writerow(['timestamp', 'source', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        
        # Register a shutdown hook to ensure the file is closed properly.
        atexit.register(self.cleanup)
        
        # Subscribers for odometry topics.
        self.create_subscription(Odometry, 'yaw_rate/odom', self.yaw_callback, 10)
        self.create_subscription(Odometry, 'single_track/odom', self.single_callback, 10)
        self.create_subscription(Odometry, 'double_track/odom', self.double_callback, 10)
        # Subscriber for Gazebo model states.
        self.create_subscription(ModelStates, '/gazebo/model_states', self.gt_callback, 10)
    
    def get_timestamp(self):
        """Return the current UNIX timestamp (seconds since epoch)."""
        return time.time()
    
    def yaw_callback(self, msg: Odometry):
        """Callback for yaw_rate/odom topic."""
        timestamp = self.get_timestamp()
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.get_logger().info(f"[YAW] Position: ({pos.x}, {pos.y}, {pos.z})")
        self.csv_writer.writerow([timestamp, 'yaw_rate', pos.x, pos.y, pos.z,
                                  ori.x, ori.y, ori.z, ori.w])
        self.csv_file.flush()
    
    def single_callback(self, msg: Odometry):
        """Callback for single_track/odom topic."""
        timestamp = self.get_timestamp()
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.get_logger().info(f"[SINGLE] Position: ({pos.x}, {pos.y}, {pos.z})")
        self.csv_writer.writerow([timestamp, 'single_track', pos.x, pos.y, pos.z,
                                  ori.x, ori.y, ori.z, ori.w])
        self.csv_file.flush()
    
    def double_callback(self, msg: Odometry):
        """Callback for double_track/odom topic."""
        timestamp = self.get_timestamp()
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.get_logger().info(f"[DOUBLE] Position: ({pos.x}, {pos.y}, {pos.z})")
        self.csv_writer.writerow([timestamp, 'double_track', pos.x, pos.y, pos.z,
                                  ori.x, ori.y, ori.z, ori.w])
        self.csv_file.flush()
    
    def gt_callback(self, msg: ModelStates):
        """
        Callback for Gazebo model states.
        Looks for a model named "limo" and records its pose.
        """
        if "limo" in msg.name:
            index = msg.name.index("limo")
            pose = msg.pose[index]
            timestamp = self.get_timestamp()
            x, y, z = pose.position.x, pose.position.y, pose.position.z
            qx, qy, qz, qw = (pose.orientation.x, pose.orientation.y, 
                              pose.orientation.z, pose.orientation.w)
            self.get_logger().info(f"[GAZEBO] 'limo' Position: ({x}, {y}, {z})")
            self.csv_writer.writerow([timestamp, 'ground_truth', x, y, z, qx, qy, qz, qw])
            self.csv_file.flush()
        else:
            self.get_logger().warn("Model 'limo' not found in ModelStates.")
    
    def cleanup(self):
        """Close the CSV file when the node shuts down."""
        try:
            self.csv_file.close()
            self.get_logger().info("CSV file closed successfully.")
        except Exception as e:
            self.get_logger().error(f"Error closing CSV file: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OdomRecorderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

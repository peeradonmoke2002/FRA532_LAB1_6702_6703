#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
import yaml
import numpy as np
import time
import csv
import os

class CSVPathSaver(Node):
    def __init__(self):
        super().__init__('csv_path_saver')

        # ✅ Subscribe to Gazebo model states
        self.create_subscription(ModelStates, '/gazebo/model_states', self.gazebo_callback, 10)

        # ✅ (ถ้าต้องการ) Load waypoints from `path.yaml`
        self.waypoints = self.load_path('~/FRA532_MobileRobot/src/FRA532_LAB1_6702_6703/path_tracking/data/path.yaml')

        # ✅ Setup CSV file for saving robot trajectory data

        save_dir = os.path.expanduser("~/FRA532_MobileRobot/src/FRA532_LAB1_6702_6703/path_tracking/path_data/record_data")
        os.makedirs(save_dir, exist_ok=True)
        self.csv_file = open(os.path.join(save_dir, "robot_path_data.csv"), "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["Time (s)", "X (m)", "Y (m)"])
        self.start_time = time.time()

        self.get_logger().info("🚀 CSV logging started. Data will be saved in robot_path_data.csv")

    def load_path(self, filename):
        """ Load waypoints from YAML file """
        try:
            with open(filename, 'r') as file:
                data = yaml.safe_load(file)
        except Exception as e:
            self.get_logger().error(f"❌ Error loading YAML file: {e}")
            return []
        if 'path' not in data:
            self.get_logger().error(f"⚠️ Key 'path' not found in {filename}")
            return []
        return np.array([(point['x'], point['y']) for point in data['path']])

    def gazebo_callback(self, msg):
        """ Receive real-time pose data from Gazebo and record trajectory to CSV """
        try:
            index = msg.name.index("limo")  # ✅ เปลี่ยน "limo" ให้ตรงกับชื่อโมเดลของคุณ
        except ValueError:
            self.get_logger().error("❌ Robot model not found in Gazebo!")
            return

        pose = msg.pose[index]
        x, y = pose.position.x, pose.position.y
        current_time = time.time() - self.start_time

        # ✅ Save timestamp, x, and y to CSV
        self.csv_writer.writerow([f"{current_time:.2f}", f"{x:.3f}", f"{y:.3f}"])
        self.csv_file.flush()  # บันทึกข้อมูลทันที

        self.get_logger().info(f"📍 Recorded: time={current_time:.2f} s, x={x:.3f}, y={y:.3f}")

    def destroy_node(self):
        # ✅ ปิดไฟล์ CSV ก่อน shutdown
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CSVPathSaver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
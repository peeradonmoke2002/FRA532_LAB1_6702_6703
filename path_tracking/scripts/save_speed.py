#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
import numpy as np
import time
import csv

class SpeedCSV(Node):
    def __init__(self):
        super().__init__('speed_csv_saver')

        # Subscribe to Gazebo model states
        self.create_subscription(ModelStates, '/gazebo/model_states', self.gazebo_callback, 10)

        # Setup CSV file for saving speed data
        self.csv_file = open("speed_data.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["Time (s)", "Speed (m/s)"])

        self.start_time = time.time()
        self.get_logger().info("🚀 CSV logging started. ข้อมูลจะถูกบันทึกใน speed_data.csv")

    def gazebo_callback(self, msg):
        """ Receive real-time pose data from Gazebo, compute raw speed, and save to CSV """
        try:
            index = msg.name.index("limo")  
        except ValueError:
            self.get_logger().error(" ไม่พบโมเดลของหุ่นยนต์ใน Gazebo!")
            return

        # Extract velocity from Gazebo message and compute raw speed
        velocity = msg.twist[index].linear
        speed = np.hypot(velocity.x, velocity.y)  # คำนวณความเร็วในแนวราบ

        # Get current time relative to start_time
        current_time = time.time() - self.start_time

        # Save raw speed and timestamp to CSV
        self.csv_writer.writerow([f"{current_time:.2f}", f"{speed:.3f}"])
        self.csv_file.flush()  # บังคับให้เขียนข้อมูลลงไฟล์ทันที

        self.get_logger().info(f"📊 Speed: {speed:.3f} m/s at {current_time:.2f} sec")

    def destroy_node(self):
        # ปิดไฟล์ CSV ก่อนปิด node
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SpeedCSV()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

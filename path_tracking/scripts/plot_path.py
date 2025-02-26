#!/usr/bin/python3

import csv
import yaml
import numpy as np
import matplotlib.pyplot as plt

def load_path(filename):
    try:
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
    except Exception as e:
        print(f"❌ Error loading YAML file: {e}")
        return np.empty((0, 2))
    
    if 'path' not in data:
        print(f"⚠️ Key 'path' not found in {filename}")
        return np.empty((0, 2))
    
    return np.array([(point['x'], point['y']) for point in data['path']])

def main():
    csv_filename = "robot_path_data.csv"
    times = []
    x_vals = []
    y_vals = []

    # อ่านข้อมูลจาก CSV สำหรับเส้นทางที่หุ่นยนต์เดินทาง
    try:
        with open(csv_filename, "r") as csv_file:
            reader = csv.DictReader(csv_file)
            for row in reader:
                times.append(float(row["Time (s)"]))
                x_vals.append(float(row["X (m)"]))
                y_vals.append(float(row["Y (m)"]))
    except FileNotFoundError:
        print(f"❌ ไม่พบไฟล์ '{csv_filename}' เลย! รัน node บันทึกข้อมูลก่อนนะครับ")
        return
    except Exception as e:
        print(f"❌ มีข้อผิดพลาดในการอ่าน CSV: {e}")
        return

    # โหลด benchmark path จาก YAML file
    yaml_filename = '/home/tang/ros2_lab1_m/src/FRA532_LAB1_6702_6703/robot_controller/data/path.yaml'
    waypoints = load_path(yaml_filename)

    # พล็อตกราฟ
    plt.figure(figsize=(8, 6))

    # พล็อต benchmark path จาก YAML (เส้นสีเขียว, เส้นหนากว่า, อยู่ด้านล่าง)
    if waypoints.size > 0:
        plt.plot(waypoints[:, 0], waypoints[:, 1], 'g-', linewidth=3, label="Benchmark Path (path.yaml)", zorder=1)  
    else:
        print("⚠️ Benchmark path is empty!")

    # พล็อตเส้นทางที่หุ่นยนต์เดินทางจาก CSV (เส้นสีแดง, เส้นบาง, อยู่ด้านบน)
    plt.plot(x_vals, y_vals, 'r-', linewidth=1, label="Robot Path", zorder=2)  

    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.title("Robot Trajectory vs Benchmark Path")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.show()

if __name__ == "__main__":
    main()

#!/usr/bin/python3

import csv
import yaml
import numpy as np
import matplotlib.pyplot as plt
import os

def load_path(filename):
    try:
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
            print(f"✅ Loaded YAML data: {data}")  # Debugging line
    except Exception as e:
        print(f"❌ Error loading YAML file: {e}")
        return np.empty((0, 2))
    
    # Check if the data is a list instead of a dictionary
    if isinstance(data, list):
        return np.array([(point['x'], point['y']) for point in data])
    
    print(f"⚠️ Invalid format in {filename}")
    return np.empty((0, 2))


def calculate_rms_errors(waypoints, robot_path):
    """Compute RMS errors for X, Y, and overall."""
    if waypoints.shape[0] == 0 or len(robot_path) == 0:
        print("⚠️ Cannot calculate RMS error: One or both paths are empty!")
        return None, None, None

    # Ensure same length (trim if needed)
    min_length = min(waypoints.shape[0], len(robot_path))
    waypoints = waypoints[:min_length]  
    robot_path = np.array(robot_path)[:min_length]

    # Compute errors
    x_errors = waypoints[:, 0] - robot_path[:, 0]
    y_errors = waypoints[:, 1] - robot_path[:, 1]

    # Compute RMS values
    rms_x = np.sqrt(np.mean(x_errors ** 2))
    rms_y = np.sqrt(np.mean(y_errors ** 2))
    rms_overall = np.sqrt(rms_x ** 2 + rms_y ** 2)

    return rms_x, rms_y, rms_overall


def main():
    base_dir = os.path.expanduser(
        "~/FRA532_MobileRobot/src/FRA532_LAB1_6702_6703/path_tracking/"
    )
    csv_filename = os.path.join(base_dir, "path_data/record_data/robot_path_data.csv")
    times = []
    x_vals = []
    y_vals = []

    # Read robot path from CSV
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

    # Load benchmark path from YAML
    yaml_filename = os.path.join(base_dir, "path_data/path.yaml")
    waypoints = load_path(yaml_filename)

    # Calculate RMS Errors
    robot_path = list(zip(x_vals, y_vals))
    rms_x, rms_y, rms_overall = calculate_rms_errors(waypoints, robot_path)

    # Plot graph
    plt.figure(figsize=(8, 6))

    # Plot benchmark path
    if waypoints.size > 0:
        plt.plot(waypoints[:, 0], waypoints[:, 1], 'g-', linewidth=3, label="Benchmark Path (path.yaml)", zorder=1)
    else:
        print("⚠️ Benchmark path is empty!")

    # Plot robot path
    plt.plot(x_vals, y_vals, 'r-', linewidth=1, label="Robot Path", zorder=2)

    # Show RMS errors in title
    if rms_overall is not None:
        plt.title(f"Robot Trajectory vs Benchmark Path\nRMS Errors - X: {rms_x:.4f} m, Y: {rms_y:.4f} m, Overall: {rms_overall:.4f} m")
    else:
        plt.title("Robot Trajectory vs Benchmark Path")

    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.show()

    # Print RMS errors
    if rms_overall is not None:
        print(f"✅ RMS Errors:")
        print(f"   - X: {rms_x:.4f} meters")
        print(f"   - Y: {rms_y:.4f} meters")
        print(f"   - Overall: {rms_overall:.4f} meters")

if __name__ == "__main__":
    main()

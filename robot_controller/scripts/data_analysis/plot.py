#!/usr/bin/python3

import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

def load_csv(file_name):
    """
    Load CSV data from a file.
    
    Returns:
        tuple: Three numpy arrays (timestamps, xs, ys).
    """
    timestamps = []
    xs = []
    ys = []
    try:
        with open(file_name, 'r') as f:
            reader = csv.reader(f)
            header = next(reader)  # Skip header row
            for row in reader:
                timestamps.append(float(row[0]))  # Convert timestamp to float
                xs.append(float(row[1]))
                ys.append(float(row[2]))
    except Exception as e:
        print(f"Error reading file {file_name}: {e}")
    
    if len(timestamps) == 0:
        print(f"⚠️ Warning: No data found in {file_name}")
    
    return np.array(timestamps), np.array(xs), np.array(ys)

def compute_rms_error(est_x, est_y, gt_x, gt_y):
    """
    Compute the Root Mean Square (RMS) error between estimated and ground truth trajectories.
    """
    errors = np.sqrt((est_x - gt_x) ** 2 + (est_y - gt_y) ** 2)
    rms_error = np.sqrt(np.mean(errors ** 2))
    return rms_error, errors

def main(): 
    path = "/home/peeradon/FRA532_MobileRobot/src/"
    # Load estimated odometry data from CSV
    est_time, est_x, est_y = load_csv(f"{path}/odom_estimated.csv")
    # Load Gazebo ground truth data
    gt_time, gt_x, gt_y = load_csv(f"{path}/odom_gazebo_limo.csv")

    # Check if data was successfully loaded
    if len(est_time) == 0 or len(gt_time) == 0:
        print("❌ Error: One or both CSV files are empty. Cannot proceed.")
        return

    # Interpolate ground truth positions to match estimated timestamps
    if len(gt_time) > 1:  # Ensure at least two points exist for interpolation
        interp_x = interp1d(gt_time, gt_x, kind="linear", fill_value="extrapolate")
        interp_y = interp1d(gt_time, gt_y, kind="linear", fill_value="extrapolate")

        gt_x_interp = interp_x(est_time)
        gt_y_interp = interp_y(est_time)
    else:
        print("❌ Error: Not enough data points in ground truth CSV for interpolation.")
        return

    # Compute RMS error
    rms_error, errors = compute_rms_error(est_x, est_y, gt_x_interp, gt_y_interp)

    # Plot trajectories
    plt.figure(figsize=(10, 8))
    plt.plot(est_x, est_y, 'bo-', label="Estimated Odometry")  # Blue line with circles
    plt.plot(gt_x, gt_y, 'rx-', label="Gazebo Ground Truth")  # Red line with crosses

    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.title(f"Trajectory Comparison: Odometry vs. Gazebo\nRMS Error: {rms_error:.4f} m")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")  # Ensure equal scaling for x and y axes

    # Show the plot
    plt.show()

    # Optionally, save the plot
    # plt.savefig("trajectory_comparison.png")

    print(f"✅ RMS Error: {rms_error:.4f} meters")

if __name__ == "__main__":
    main()

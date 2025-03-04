#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import csv

def load_csv(filename):
    """
    Load data from a CSV file and convert it into a numpy array.
    Assumes the file has a header.
    """
    data = []
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)  # skip header
        for row in reader:
            data.append([float(val) for val in row])
    return np.array(data)

def calculate_rmse(ekf, gps):
    """
    Calculate the Root Mean Square Error (RMSE) between EKF and GPS trajectories.
    """
    min_len = min(len(ekf), len(gps))
    ekf = ekf[:min_len, :]
    gps = gps[:min_len, :]
    rmse = np.sqrt(np.mean((ekf[:, 0] - gps[:, 0]) ** 2 + (ekf[:, 1] - gps[:, 1]) ** 2))
    return rmse

def calculate_mae(ekf, gps):
    """
    Calculate the Mean Absolute Error (MAE) between EKF and GPS trajectories.
    """
    min_len = min(len(ekf), len(gps))
    ekf = ekf[:min_len, :]
    gps = gps[:min_len, :]
    mae = np.mean(np.abs(ekf[:, 0] - gps[:, 0]) + np.abs(ekf[:, 1] - gps[:, 1]))
    return mae

def compute_percent_change(wheel_angles):
    """
    Compute the percentage change of the steering angles (from front steering)
    by comparing the current value with the previous value.
    Returns two arrays: one for the front left and one for the front right.
    """
    left_changes = []
    right_changes = []
    for i in range(1, len(wheel_angles)):
        prev_left, prev_right = wheel_angles[i-1]
        cur_left, cur_right = wheel_angles[i]
        left_change = abs(cur_left - prev_left) / abs(prev_left) * 100 if prev_left != 0 else 0
        right_change = abs(cur_right - prev_right) / abs(prev_right) * 100 if prev_right != 0 else 0
        left_changes.append(left_change)
        right_changes.append(right_change)
    return np.array(left_changes), np.array(right_changes)

def calculate_slip(wheel_angles):
    """
    Calculate the slip percentage for the wheels (assumed to be the rear wheels)
    using the formula:
        slip = (|left - right| / average(|left|, |right|)) * 100
    Returns an array of slip values for each measurement.
    """
    slips = []
    for left, right in wheel_angles:
        avg = (abs(left) + abs(right)) / 2.0
        slip = (abs(left - right) / avg * 100) if avg != 0 else 0
        slips.append(slip)
    return np.array(slips)

def main():
    # Load CSV data:
    dr_data = load_csv("dead_reckoning.csv")    # Dead Reckoning trajectory [x, y]
    ekf_data = load_csv("ekf_estimated.csv")      # EKF Estimated trajectory [x, y]
    gps_data = load_csv("gps_obs.csv")            # GPS observations [x, y]
    wheel_data = load_csv("front_wheel_angles.csv")     # Wheel angles for front steering: [front_left, front_right]
    # Load slip data from rear_slip.csv (each row contains a single slip percentage)
    slip_data = load_csv("rear_slip.csv")         # Rear wheel slip percentages

    # Calculate RMSE and MAE between EKF and GPS trajectories
    rmse = calculate_rmse(ekf_data, gps_data)
    mae = calculate_mae(ekf_data, gps_data)
    print("RMSE (EKF vs GPS):", rmse)
    print("MAE (EKF vs GPS):", mae)

    # Compute percent change for front steering angles
    left_pct, right_pct = compute_percent_change(wheel_data)
    avg_left_pct = np.mean(left_pct) if len(left_pct) > 0 else 0
    avg_right_pct = np.mean(right_pct) if len(right_pct) > 0 else 0
    print("Average percent change for front left steering:", avg_left_pct, "%")
    print("Average percent change for front right steering:", avg_right_pct, "%")

    # Calculate average slip percentage from rear_slip.csv data
    # Assume slip_data is a single-column array, so flatten it first.
    slip_data = slip_data.flatten()
    avg_slip = np.mean(slip_data) if len(slip_data) > 0 else 0
    print("Average slip (%):", avg_slip)

    # Plot two subplots in one figure
    plt.figure(figsize=(12, 10))

    # Subplot 1: Trajectory Comparison (Dead Reckoning, GPS, EKF)
    plt.subplot(2, 1, 1)
    if dr_data.shape[0] > 0:
        plt.plot(dr_data[:, 0], dr_data[:, 1], 'k--', label="Dead Reckoning")
    if gps_data.shape[0] > 0:
        plt.plot(gps_data[:, 0], gps_data[:, 1], 'g.', label="GPS Observations")
    if ekf_data.shape[0] > 0:
        plt.plot(ekf_data[:, 0], ekf_data[:, 1], 'r-', label="EKF Estimated Trajectory")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.title("Trajectory Comparison (EKF vs GPS vs Dead Reckoning)")
    plt.legend()
    plt.grid(True)
    # Annotate RMSE and MAE on the trajectory plot in red text
    plt.text(0.05, 0.95, f"RMSE: {rmse:.3f}\nMAE: {mae:.3f}", transform=plt.gca().transAxes, 
             fontsize=12, color='red', verticalalignment='top', bbox=dict(facecolor='white', alpha=0.8))

    # Subplot 2: Plot percent change of front steering angles and rear slip percentage
    plt.subplot(2, 1, 2)
    indices_front = np.arange(len(left_pct))
    if left_pct.size > 0:
        plt.plot(indices_front, left_pct, 'b-', label="Front Left % Change")
    if right_pct.size > 0:
        plt.plot(indices_front, right_pct, 'm-', label="Front Right % Change")
    # Plot slip data from rear wheels
    indices_slip = np.arange(len(slip_data))
    if slip_data.size > 0:
        plt.plot(indices_slip, slip_data, 'r-', label="Rear Wheel Slip (%)")
    plt.xlabel("Measurement Index")
    plt.ylabel("Percent (%)")
    plt.title("Wheel Steering Angle % Change and Rear Wheel Slip")
    plt.legend()
    plt.grid(True)
    # Annotate average values for front steering percent change and rear slip in red text
    plt.text(0.05, 0.95, f"Avg Front Left % Change: {avg_left_pct:.1f}%\nAvg Front Right % Change: {avg_right_pct:.1f}%\nAvg Rear Slip: {avg_slip:.1f}%", 
             transform=plt.gca().transAxes, fontsize=12, color='red', verticalalignment='top', bbox=dict(facecolor='white', alpha=0.8))

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()

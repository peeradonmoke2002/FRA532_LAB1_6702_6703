#!/usr/bin/env python3

import os
import pandas as pd
import matplotlib.pyplot as plt
import yaml
import numpy as np

def load_waypoints(file_path):
    """Load waypoints from a YAML file."""
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    waypoints = [(wp['x'], wp['y'], wp.get('yaw', 0.0)) for wp in data]
    print(f"Loaded {len(waypoints)} waypoints from {file_path}.")
    return waypoints

def compute_rmse_axis(ref_points, recorded_points):
    """
    Compute the RMSE between the reference points and recorded points.
    For each recorded point, the error is defined by its nearest reference point.
    
    Returns overall_rmse, rmse_x, rmse_y.
    """
    errors_x = []
    errors_y = []
    for rp in recorded_points:
        # Compute Euclidean distances to every reference point.
        dists = np.linalg.norm(ref_points - rp, axis=1)
        idx = np.argmin(dists)
        error_x = rp[0] - ref_points[idx, 0]
        error_y = rp[1] - ref_points[idx, 1]
        errors_x.append(error_x**2)
        errors_y.append(error_y**2)
    rmse_x = np.sqrt(np.mean(errors_x))
    rmse_y = np.sqrt(np.mean(errors_y))
    overall_rmse = np.sqrt(np.mean(np.array(errors_x) + np.array(errors_y)))
    return overall_rmse, rmse_x, rmse_y

def plot_speed_profile(joint_states_csv):
    """
    Plot the speed profile using the rear wheel speeds from the joint_states CSV.
    Computes an average speed from left and right wheels.
    """
    # Read joint states data; expected columns: time, v_rl, v_rr, delta_fl, delta_fr
    df = pd.read_csv(joint_states_csv)
    # Compute average wheel speed.
    df['v_avg'] = (df['v_rl'] + df['v_rr']) / 2.0

    plt.figure()
    plt.plot(df['time'], df['v_rl'], label='Rear Left Wheel Speed', alpha=0.7)
    plt.plot(df['time'], df['v_rr'], label='Rear Right Wheel Speed', alpha=0.7)
    plt.plot(df['time'], df['v_avg'], 'k--', label='Average Speed', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')
    plt.title('Speed Profile')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def plot_trajectory(model_states_csv, ref_path_yaml=None):
    """
    Plot the recorded XY trajectory (from model_states CSV) and overlay the reference path.
    The CSV trajectory will be plotted on top.
    Additionally, compute and display the RMSE overall and in each axis.
    """
    plt.figure()
    
    # First, plot the reference path (if available) in the background.
    if ref_path_yaml and os.path.exists(ref_path_yaml):
        waypoints = load_waypoints(ref_path_yaml)
        xs = [wp[0] for wp in waypoints]
        ys = [wp[1] for wp in waypoints]
        plt.plot(xs, ys, 'ro-', label='Reference Path (YAML)')
    else:
        print("Reference path file not found. Skipping overlay.")
        # If no reference, we set rmse values to None.
        xs, ys = None, None
    
    # Now, plot the recorded trajectory so it appears on top.
    df = pd.read_csv(model_states_csv)
    plt.plot(df['x'], df['y'], 'b-', label='Recorded Trajectory', linewidth=2)
    
    title_str = "Trajectory vs. Reference Path"
    # Compute RMSE if reference path is available.
    if xs is not None and ys is not None:
        ref_points = np.array(list(zip(xs, ys)))
        recorded_points = df[['x', 'y']].to_numpy()
        overall_rmse, rmse_x, rmse_y = compute_rmse_axis(ref_points, recorded_points)
        title_str += f", Overall RMSE = {overall_rmse:.3f} m\nRMSE X = {rmse_x:.3f} m, RMSE Y = {rmse_y:.3f} m"
        print("Computed RMSE values:")
        print(f"Overall RMSE: {overall_rmse:.3f} m")
        print(f"RMSE in X: {rmse_x:.3f} m")
        print(f"RMSE in Y: {rmse_y:.3f} m")
    
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title(title_str)
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def main():
    # Base directory where the recorder node saved CSV files.
    base_dir = os.path.expanduser(
        "~/FRA532_MobileRobot/src/FRA532_LAB1_6702_6703/path_tracking/path_data/record_data"
    )
    joint_states_csv = os.path.join(base_dir, 'joint_states_data.csv')
    model_states_csv = os.path.join(base_dir, 'model_states_data.csv')
    
    # Reference path YAML file.
    ref_path_yaml = os.path.join(
        os.path.expanduser("~/FRA532_MobileRobot/src/FRA532_LAB1_6702_6703/path_tracking/path_data"),
        'path.yaml'
    )

    # Check if recorded CSV files exist.
    if not os.path.exists(joint_states_csv) or not os.path.exists(model_states_csv):
        print("Error: One or more CSV files were not found. "
              "Ensure that the data recording node has saved the CSV files correctly.")
        return

    # Plot the speed profile from joint states.
    plot_speed_profile(joint_states_csv)

    # Plot the trajectory and overlay the reference path, then compute RMSE.
    plot_trajectory(model_states_csv, ref_path_yaml=ref_path_yaml)

if __name__ == '__main__':
    main()

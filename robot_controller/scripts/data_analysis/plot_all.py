#!/usr/bin/env python3

import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def compute_rmse(odo_df, gt_df):
    """
    Compute the RMSE between an odometry dataframe and a ground truth dataframe.
    Both dataframes must be sorted by timestamp.
    Uses merge_asof to align the data on timestamp.
    """
    merged = pd.merge_asof(odo_df.sort_values('timestamp'),
                           gt_df.sort_values('timestamp'),
                           on='timestamp', suffixes=('', '_gt'))
    # Compute 2D Euclidean error between odometry and ground truth positions
    merged['error'] = np.sqrt((merged['x'] - merged['x_gt'])**2 +
                              (merged['y'] - merged['y_gt'])**2)
    rmse = np.sqrt(np.mean(merged['error']**2))
    return rmse

def main():
    # Update the CSV file path to match your saved file location
    csv_file_path = os.path.expanduser("~/FRA532_MobileRobot/src/FRA532_LAB1_6702_6703/robot_controller/scripts/data_analysis/record_data/odometry_data.csv")
    
    if not os.path.exists(csv_file_path):
        print(f"CSV file not found: {csv_file_path}")
        return
    
    # Load CSV data
    df = pd.read_csv(csv_file_path)
    
    # Sort by timestamp
    df.sort_values('timestamp', inplace=True)
    
    # Get unique source names
    sources = df['source'].unique()
    
    plt.figure(figsize=(10, 8))
    
    # Define distinct marker symbols and colors for each source
    markers = {
        'yaw_rate': 'o',         # circle
        'single_track': 's',     # square
        'double_track': '^',     # triangle_up
        'ground_truth': 'x'      # x-mark
    }
    colors = {
        'yaw_rate': 'blue',
        'single_track': 'green',
        'double_track': 'red',
        'ground_truth': 'purple'
    }
    
    for src in sources:
        df_src = df[df['source'] == src]
        # Scatter plot with markers
        plt.scatter(
            df_src['x'], df_src['y'], 
            label=src, 
            marker=markers.get(src, 'o'), 
            color=colors.get(src, None), 
            alpha=0.8, 
            s=40  # marker size
        )
        # Add a dotted line connecting the points
        plt.plot(
            df_src['x'], df_src['y'], 
            linestyle='--', 
            color=colors.get(src, None), 
            alpha=0.5
        )
    
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.title("XY Trajectories for Odometry Models")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()
    
    # Compute RMSE for each odometry model relative to ground truth
    df_gt = df[df['source'] == 'ground_truth']
    for src in ['yaw_rate', 'single_track', 'double_track']:
        df_src = df[df['source'] == src]
        if df_src.empty or df_gt.empty:
            print(f"No data available for {src} or ground truth to compute RMSE.")
            continue
        rmse = compute_rmse(df_src, df_gt)
        print(f"RMSE for {src} relative to ground truth: {rmse:.4f}")

if __name__ == '__main__':
    main()

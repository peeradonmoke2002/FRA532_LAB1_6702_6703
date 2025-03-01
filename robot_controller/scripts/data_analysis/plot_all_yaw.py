#!/usr/bin/env python3

import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def quaternion_to_yaw(qx, qy, qz, qw):
    """
    Convert quaternion (qx, qy, qz, qw) to yaw (theta).
    Assumes a planar movement (only yaw is relevant).
    """
    yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 
                     1.0 - 2.0 * (qy**2 + qz**2))
    return yaw

def compute_rmse(odo_df, gt_df):
    """
    Compute the RMSE between odometry and ground truth yaw.
    Uses merge_asof to align timestamps before computing RMSE.
    """
    merged = pd.merge_asof(odo_df.sort_values('timestamp'),
                           gt_df.sort_values('timestamp'),
                           on='timestamp', suffixes=('', '_gt'))
    merged['error'] = (merged['theta'] - merged['theta_gt'])**2
    rmse = np.sqrt(np.mean(merged['error']))
    return rmse

def main():
    # Update the CSV file path
    csv_file_path = os.path.expanduser("~/FRA532_MobileRobot/src/FRA532_LAB1_6702_6703/robot_controller/scripts/data_analysis/record_data/odometry_data.csv")
    
    if not os.path.exists(csv_file_path):
        print(f"CSV file not found: {csv_file_path}")
        return
    
    # Load CSV data
    df = pd.read_csv(csv_file_path)

    # Ensure required quaternion columns exist
    if not {'qx', 'qy', 'qz', 'qw'}.issubset(df.columns):
        print("Quaternion columns (qx, qy, qz, qw) not found in CSV.")
        return
    
    # Convert quaternions to yaw (theta)
    df['theta'] = df.apply(lambda row: quaternion_to_yaw(row['qx'], row['qy'], row['qz'], row['qw']), axis=1)

    # Sort by timestamp
    df.sort_values('timestamp', inplace=True)

    ### **Plot Yaw Orientation (Theta) Over Time**
    plt.figure(figsize=(10, 6))

    # Define colors for plotting
    colors = {
        'yaw_rate': 'blue',
        'single_track': 'green',
        'double_track': 'red',
        'ground_truth': 'purple'
    }

    # Plot yaw orientation for each source
    for src in ['yaw_rate', 'single_track', 'double_track', 'ground_truth']:
        df_src = df[df['source'] == src]
        if not df_src.empty:
            plt.plot(df_src['timestamp'], df_src['theta'], 
                     label=f"{src} Heading", 
                     color=colors.get(src, None), 
                     linestyle='--' if src != 'ground_truth' else '-',
                     marker='o',
                     alpha=0.8)

    plt.xlabel("Timestamp (s)")
    plt.ylabel("Yaw Orientation (radians)")
    plt.title("Yaw Orientation (Heading) Over Time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    ### **Compute RMSE for Each Model Relative to Ground Truth**
    df_gt = df[df['source'] == 'ground_truth'].rename(columns={'theta': 'theta_gt'})

    for src in ['yaw_rate', 'single_track', 'double_track']:
        df_src = df[df['source'] == src]
        if df_src.empty or df_gt.empty:
            print(f"No data available for {src} or ground truth to compute RMSE.")
            continue
        
        # Compute RMSE for yaw orientation (theta)
        rmse_yaw = compute_rmse(df_src, df_gt)
        print(f"RMSE for {src} (yaw orientation) relative to ground truth: {rmse_yaw:.4f} radians")

if __name__ == '__main__':
    main()

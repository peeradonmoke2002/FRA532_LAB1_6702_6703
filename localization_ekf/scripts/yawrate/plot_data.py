#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

def calculate_rmse(ekf, gps):
    """ Compute Root Mean Square Error (RMSE) between EKF and GPS. """
    if ekf is None or gps is None:
        return None
    
    min_len = min(len(ekf), len(gps))
    ekf, gps = ekf[:min_len], gps[:min_len]

    rmse = np.sqrt(np.mean((ekf[:, 0] - gps[:, 0]) ** 2 + (ekf[:, 1] - gps[:, 1]) ** 2))
    return rmse

def calculate_mae(ekf, gps):
    """ Compute Mean Absolute Error (MAE) between EKF and GPS. """
    if ekf is None or gps is None:
        return None
    
    min_len = min(len(ekf), len(gps))
    ekf, gps = ekf[:min_len], gps[:min_len]

    mae = np.mean(np.abs(ekf[:, 0] - gps[:, 0]) + np.abs(ekf[:, 1] - gps[:, 1]))
    return mae

def load_and_analyze():
    # Load CSV files.
    try:
        dr = np.loadtxt("dead_reckoning.csv", delimiter=",", skiprows=1)
    except Exception as e:
        print("Error loading dead_reckoning.csv:", e)
        dr = None

    try:
        gps = np.loadtxt("gps_obs.csv", delimiter=",", skiprows=1)
    except Exception as e:
        print("Error loading gps_obs.csv:", e)
        gps = None

    try:
        ekf = np.loadtxt("ekf_estimated.csv", delimiter=",", skiprows=1)
    except Exception as e:
        print("Error loading ekf_estimated.csv:", e)
        ekf = None

    # Compute errors
    rmse = calculate_rmse(ekf, gps)
    mae = calculate_mae(ekf, gps)

    # Print error values
    if rmse is not None:
        print(f"RMSE (EKF vs GPS): {rmse:.4f} meters")
    else:
        print("RMSE calculation failed.")

    if mae is not None:
        print(f"MAE (EKF vs GPS): {mae:.4f} meters")
    else:
        print("MAE calculation failed.")

    # Create the plot
    plt.figure(figsize=(10, 6))

    if dr is not None and dr.size:
        plt.plot(dr[:, 0], dr[:, 1], 'k--', label="Dead Reckoning")
    if gps is not None and gps.size:
        plt.scatter(gps[:, 0], gps[:, 1], color='g', marker='o', label="GPS Observations")
    if ekf is not None and ekf.size:
        plt.plot(ekf[:, 0], ekf[:, 1], 'r-', label="EKF Estimated Trajectory")

    # Add RMSE and MAE as text annotations
    # error_text = f"RMSE: {rmse:.4f} m\nMAE: {mae:.4f} m" if rmse is not None and mae is not None else "Error metrics unavailable"
    # plt.text(0.05, 0.95, error_text, transform=plt.gca().transAxes, fontsize=12,
    #          verticalalignment='top', bbox=dict(facecolor='white', alpha=0.7, edgecolor='black'))

    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.title("EKF Trajectory Analysis with RMSE & MAE [STANLEE-doubletrack-crash]")
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    load_and_analyze()

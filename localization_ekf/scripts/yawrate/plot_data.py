#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

def load_and_plot():
    # Load CSV files.
    # Note: The CSV files are assumed to have a header line ("x,y").
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

    # Create the plot
    plt.figure(figsize=(10, 6))

    if dr is not None and dr.size:
        plt.plot(dr[:, 0], dr[:, 1], 'k--', label="Dead Reckoning")
    if gps is not None and gps.size:
        plt.scatter(gps[:, 0], gps[:, 1], color='g', marker='o', label="GPS Observations")
    if ekf is not None and ekf.size:
        plt.plot(ekf[:, 0], ekf[:, 1], 'r-', label="EKF Estimated Trajectory")

    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.title(" Trajectories from EKF Localization [SingleTrack-PID]")
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    load_and_plot()

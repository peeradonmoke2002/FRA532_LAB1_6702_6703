import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicHermiteSpline    
import math


# Example: suppose your waypoints have [x, y, yaw] (yaw in radians)
waypoints = np.array([
    [0, 0, 0],
    [1, -2, np.deg2rad(0)],
    [2, -2, np.deg2rad(0)],
    [-3, 3, np.deg2rad(-45)],
    [4, -4, np.deg2rad(-90)]
])

# 1. Compute the cumulative distance (t parameter) along the waypoints
t = np.zeros(waypoints.shape[0])
for i in range(1, len(t)):
    t[i] = t[i-1] + np.linalg.norm(waypoints[i, :2] - waypoints[i-1, :2])

# 2. Use the yaw to compute the derivative at each waypoint.
# Here, the derivative direction is (cos(yaw), sin(yaw)).
# To set the magnitude, we use a scale factor based on the chord length.
dxdt = []
dydt = []
for i in range(len(waypoints)):
    # For endpoints, use the difference with the nearest neighbor.
    if i == 0:
        d = t[i+1] - t[i]
    elif i == len(waypoints)-1:
        d = t[i] - t[i-1]
    else:
        d = (t[i+1] - t[i-1]) / 2.0
    dxdt.append(np.cos(waypoints[i, 2]) * d)
    dydt.append(np.sin(waypoints[i, 2]) * d)
dxdt = np.array(dxdt)
dydt = np.array(dydt)

# 3. Create Hermite splines for x(t) and y(t)
spline_x = CubicHermiteSpline(t, waypoints[:, 0], dxdt)
spline_y = CubicHermiteSpline(t, waypoints[:, 1], dydt)

# 4. Evaluate the splines for a smooth curve
t_smooth = np.linspace(t[0], t[-1], 200)
x_smooth = spline_x(t_smooth)
y_smooth = spline_y(t_smooth)

# 5. Plot the results
plt.figure()
plt.plot(waypoints[:, 0], waypoints[:, 1], 'ro', label='Waypoints')
plt.plot(x_smooth, y_smooth, 'b-', label='Fitted Curve')
# Plot yaw directions at each waypoint for reference
plt.quiver(waypoints[:, 0], waypoints[:, 1], 
           np.cos(waypoints[:, 2]), np.sin(waypoints[:, 2]), 
           color='g', scale=5, label='Yaw Direction')
plt.xlabel("x")
plt.ylabel("y")
plt.title("Curve Fitting using Path Yaw Points")
plt.legend()
plt.show()

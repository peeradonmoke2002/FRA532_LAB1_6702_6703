#!/usr/bin/python3
"""
This ROS2 node implements iterative linear MPC for path tracking
by porting Atsushi Sakai's PythonRobotics code.
It reads waypoints from a YAML file, computes a reference trajectory,
solves the MPC problem using cvxpy, and publishes steering and speed commands.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from gazebo_msgs.msg import ModelStates
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
import tf_transformations
import math
import numpy as np
import yaml
import cvxpy
import matplotlib.pyplot as plt
import time

# ----- Global Constants (from the simulation code) -----
NX = 4  # [x, y, v, yaw]
NU = 2  # [accel, steer]
T = 5   # Prediction horizon length
DT = 0.2  # [s] time tick for the MPC prediction

# MPC cost matrices
R = np.diag([0.01, 0.01])       # Input cost matrix
Rd = np.diag([0.01, 1.0])         # Input difference cost matrix
Q = np.diag([1.0, 1.0, 0.5, 0.5])  # State cost matrix
Qf = Q                          # Final state cost

# Iterative MPC parameters
MAX_ITER = 3      # Maximum iteration for updating the operating point
DU_TH = 0.1       # Termination threshold on input change

TARGET_SPEED = 10.0 / 3.6  # [m/s] target speed (10 km/h)

N_IND_SEARCH = 10  # Number of waypoints to search for nearest index

# Vehicle parameters
WB = 2.5  # Wheel base [m]
MAX_STEER = np.deg2rad(45.0)  # Maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(30.0)   # Maximum steering rate [rad/s]
MAX_SPEED = 55.0 / 3.6  # Maximum speed [m/s]
MIN_SPEED = -20.0 / 3.6  # Minimum speed [m/s]
MAX_ACCEL = 1.0         # Maximum acceleration [m/s^2]

# ----- Helper Classes and Functions -----
class State:
    """Vehicle state class."""
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

def pi_2_pi(angle):
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def get_linear_model_matrix(v, phi, delta):
    """
    Linearize the bicycle model around the current operating point.
    Returns matrices A, B, and a constant term C.
    """
    A = np.zeros((NX, NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0
    A[0, 2] = DT * math.cos(phi)
    A[0, 3] = -DT * v * math.sin(phi)
    A[1, 2] = DT * math.sin(phi)
    A[1, 3] = DT * v * math.cos(phi)
    A[3, 2] = DT * math.tan(delta) / WB

    B = np.zeros((NX, NU))
    B[2, 0] = DT
    B[3, 1] = DT * v / (WB * math.cos(delta) ** 2)

    C = np.zeros(NX)
    C[0] = DT * v * math.sin(phi) * phi
    C[1] = -DT * v * math.cos(phi) * phi
    C[3] = -DT * v * delta / (WB * math.cos(delta) ** 2)

    return A, B, C

def update_state(state, a, delta):
    """
    Update the state given control inputs (for prediction purposes).
    """
    # Saturate steering angle
    if delta >= MAX_STEER:
        delta = MAX_STEER
    elif delta <= -MAX_STEER:
        delta = -MAX_STEER

    state.x += state.v * math.cos(state.yaw) * DT
    state.y += state.v * math.sin(state.yaw) * DT
    state.yaw += state.v / WB * math.tan(delta) * DT
    state.v += a * DT

    if state.v > MAX_SPEED:
        state.v = MAX_SPEED
    elif state.v < MIN_SPEED:
        state.v = MIN_SPEED

    return state

def get_nparray_from_matrix(x):
    return np.array(x).flatten()

def predict_motion(x0, oa, od, xref):
    """
    Predict state trajectory (operating point) using previous inputs.
    """
    xbar = xref * 0.0
    for i in range(NX):
        xbar[i, 0] = x0[i]

    state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
    for i in range(1, T + 1):
        a_i = oa[i-1] if oa is not None else 0.0
        d_i = od[i-1] if od is not None else 0.0
        state = update_state(state, a_i, d_i)
        xbar[0, i] = state.x
        xbar[1, i] = state.y
        xbar[2, i] = state.v
        xbar[3, i] = state.yaw

    return xbar

def linear_mpc_control(xref, xbar, x0, dref):
    """
    Solve the linear MPC problem via cvxpy.
    xref: Reference trajectory (NX x T+1)
    xbar: Predicted trajectory (NX x T+1)
    x0: Initial state vector [x, y, v, yaw]
    dref: Reference steering (1 x T+1)
    """
    x = cvxpy.Variable((NX, T + 1))
    u = cvxpy.Variable((NU, T))
    cost = 0.0
    constraints = []

    for t in range(T):
        cost += cvxpy.quad_form(u[:, t], R)
        if t != 0:
            cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)
        A, B, C = get_linear_model_matrix(xbar[2, t], xbar[3, t], dref[0, t])
        constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]
        if t < T - 1:
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
            constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <= MAX_DSTEER * DT]

    cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)
    constraints += [x[:, 0] == x0]
    constraints += [x[2, :] <= MAX_SPEED]
    constraints += [x[2, :] >= MIN_SPEED]
    constraints += [cvxpy.abs(u[0, :]) <= MAX_ACCEL]
    constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    # Using the ECOS solver (change if necessary)
    prob.solve(solver=cvxpy.ECOS, verbose=False)

    if prob.status in [cvxpy.OPTIMAL, cvxpy.OPTIMAL_INACCURATE]:
        ox = get_nparray_from_matrix(x.value[0, :])
        oy = get_nparray_from_matrix(x.value[1, :])
        ov = get_nparray_from_matrix(x.value[2, :])
        oyaw = get_nparray_from_matrix(x.value[3, :])
        oa = get_nparray_from_matrix(u.value[0, :])
        odelta = get_nparray_from_matrix(u.value[1, :])
    else:
        print("Error: Cannot solve mpc..")
        oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

    return oa, odelta, ox, oy, oyaw, ov

def iterative_linear_mpc_control(xref, x0, dref, prev_oa, prev_od):
    """
    Iterative linear MPC control that updates the operating point.
    """
    if prev_oa is None or prev_od is None:
        prev_oa = [0.0] * T
        prev_od = [0.0] * T

    for i in range(MAX_ITER):
        xbar = predict_motion(x0, prev_oa, prev_od, xref)
        poa, pod = prev_oa[:], prev_od[:]
        oa, odelta, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, dref)
        du = sum(abs(np.array(oa) - np.array(poa))) + sum(abs(np.array(odelta) - np.array(pod)))
        if du <= DU_TH:
            break
        prev_oa, prev_od = oa, odelta
    else:
        print("Iterative MPC reached maximum iterations")
    return oa, odelta, ox, oy, oyaw, ov

def calc_nearest_index(state, cx, cy, cyaw, pind):
    """
    Find the index of the waypoint closest to the current state.
    """
    dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]
    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]
    mind = min(d)
    ind = d.index(mind) + pind
    mind = math.sqrt(mind)
    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y
    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1
    return ind, mind

def calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, pind):
    """
    Calculate the reference trajectory for the MPC.
    Returns:
      xref: Reference state trajectory (NX x T+1)
      ind: Updated nearest waypoint index
      dref: Reference steering (1 x T+1) [here set to zeros]
    """
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))
    ncourse = len(cx)
    ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind)
    if pind >= ind:
        ind = pind

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp[ind]
    xref[3, 0] = cyaw[ind]
    dref[0, 0] = 0.0  # steering reference is zero

    travel = 0.0
    for i in range(T + 1):
        travel += abs(state.v) * DT
        dind = int(round(travel / dl))
        if (ind + dind) < ncourse:
            xref[0, i] = cx[ind + dind]
            xref[1, i] = cy[ind + dind]
            xref[2, i] = sp[ind + dind]
            xref[3, i] = cyaw[ind + dind]
            dref[0, i] = 0.0
        else:
            xref[0, i] = cx[-1]
            xref[1, i] = cy[-1]
            xref[2, i] = sp[-1]
            xref[3, i] = cyaw[-1]
            dref[0, i] = 0.0

    return xref, ind, dref

# ----- ROS2 Node with MPC -----
class LinearMPC(Node):
    def __init__(self):
        super().__init__('linear_mpc')
        
        # Publishers for steering and wheel speed commands.
        self.pub_steering = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_position_controller/joint_trajectory', 
            10
        )
        self.pub_wheel_spd = self.create_publisher(
            Float64MultiArray, 
            '/velocity_controllers/commands', 
            10
        )
        
        # Subscribe to Gazebo model states.
        self.subscription_gazebo = self.create_subscription(
            ModelStates, 
            '/gazebo/model_states', 
            self.model_states_callback, 
            10
        )
        
        # Timer to run the MPC control loop.
        self.timer = self.create_timer(0.01, self.linear_mpc_control)

        # Load waypoints from a YAML file.
        wp = self.load_waypoints(
            '/home/peeradon/FRA532_MobileRobot/src/FRA532_LAB1_6702_6703/path_controller/config/path.yaml'
        )
        wp = np.array(wp)  # Convert list to NumPy array
        self.waypoints = wp  # Each row: [x, y, yaw]
        # Extract course arrays.
        self.cx = self.waypoints[:, 0].tolist()
        self.cy = self.waypoints[:, 1].tolist()
        self.cyaw = self.waypoints[:, 2].tolist()
        self.ck = [0.0] * len(self.cx)  # For simplicity, assume zero curvature.
        self.sp = [TARGET_SPEED] * len(self.cx)
        self.dl = 1.0  # course distance resolution [m]
        self.pind = 0  # initial target index

        # Current state variables.
        self.position = (0.0, 0.0)
        self.yaw = 0.0
        self.v = 0.0  # Current speed (to be updated from twist)

        # For tracking the actual path.
        self.robot_path = []

        # Previous control inputs (for iterative MPC).
        self.oa = None
        self.odelta = None

        # Robot physical parameters.
        self.wheel_base = 0.2         # [m]
        self.track_width = 0.14       # [m]
        self.max_steering_angle = 0.523598767  # 30° in radians
        self.wheel_radius = 0.045     # [m]

        # Setup real-time plotting.
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 6))

    def load_waypoints(self, file_path):
        """Load waypoints from a YAML file."""
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
        # Expect each entry to have 'x', 'y', and optionally 'yaw'
        waypoints = [(wp['x'], wp['y'], wp.get('yaw', 0.0)) for wp in data]
        self.get_logger().info(f"Loaded {len(waypoints)} waypoints.")
        return waypoints

    def model_states_callback(self, msg):
        """Update the robot's position, yaw, and speed from Gazebo ModelStates."""
        if "limo" in msg.name:
            index = msg.name.index("limo")
            pose = msg.pose[index]
            twist = msg.twist[index]
            self.position = (pose.position.x, pose.position.y)
            # Convert quaternion to yaw.
            orientation_q = pose.orientation
            siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
            cosy_cosp = 1 - 2 * (orientation_q.y**2 + orientation_q.z**2)
            self.yaw = math.atan2(siny_cosp, cosy_cosp)
            # Update speed from twist (assuming forward speed along x).
            self.v = twist.linear.x
        else:
            self.get_logger().warn("Model 'limo' not found in ModelStates.")

    def publish_steering(self, steering: float):
        """
        Publish a steering command as a JointTrajectory message.
        The same steering angle is applied to both front steering joints.
        """
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['front_left_steering', 'front_right_steering']
        
        point = JointTrajectoryPoint()
        point.positions = [float(steering), float(steering)]
        point.time_from_start = Duration(seconds=0.1).to_msg()
        traj_msg.points.append(point)
        self.pub_steering.publish(traj_msg)

    def linear_mpc_control(self):
        """Timer callback to compute and publish MPC commands."""
        # Form current state vector: [x, y, v, yaw]
        x0 = [self.position[0], self.position[1], self.v, self.yaw]
        current_state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
        
        # Calculate reference trajectory and steering reference using current state.
        xref, self.pind, dref = calc_ref_trajectory(current_state, self.cx, self.cy, self.cyaw, self.ck, self.sp, self.dl, self.pind)
        
        # Solve MPC (iterative linear MPC).
        oa, odelta, ox, oy, oyaw, ov = iterative_linear_mpc_control(xref, x0, dref, self.oa, self.odelta)
        if oa is None or odelta is None:
            self.get_logger().warn("MPC failed to solve.")
            return
        
        # Update stored control inputs.
        self.oa = oa
        self.odelta = odelta

        # Use first control input from the solution.
        a_cmd = oa[0]
        steer_cmd = odelta[0]
        # Compute desired speed from current speed and acceleration command.
        desired_speed = self.v + a_cmd * DT
        desired_speed = max(min(desired_speed, MAX_SPEED), MIN_SPEED)
        
        # Publish steering and wheel speed commands.
        self.publish_steering(steer_cmd)
        speed_msg = Float64MultiArray()
        speed_msg.data = [desired_speed]
        self.pub_wheel_spd.publish(speed_msg)
        
        # Record robot path for plotting.
        self.robot_path.append((self.position[0], self.position[1]))
        self.ax.cla()
        self.ax.plot(self.cx, self.cy, '-r', label="Course")
        self.ax.plot([p[0] for p in self.robot_path], [p[1] for p in self.robot_path], '-g', label="Trajectory")
        if ox is not None and oy is not None:
            self.ax.plot(ox, oy, 'xr', label="MPC Prediction")
        self.ax.legend()
        self.ax.set_title(f"Speed: {self.v * 3.6:.2f} km/h")
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = LinearMPC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Linear MPC node.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

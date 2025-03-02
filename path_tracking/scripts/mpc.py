#!/usr/bin/python3

import math
import time
import numpy as np
import cvxpy
import matplotlib.pyplot as plt
import yaml
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates

from ament_index_python.packages import get_package_share_directory
from PythonRobotics.utils.angle import angle_mod
from PythonRobotics.PathPlanning.CubicSpline import cubic_spline_planner

# ==== MPC & Simulation Parameters ====
NX = 4          # State dimension: [x, y, v, yaw]
NU = 2          # Control input: [acceleration, steering]
T = 5           # Prediction horizon length
DT = 0.2        # Time tick [s]
MAX_ITER = 3    # Maximum MPC iterations
DU_TH = 0.1     # Convergence threshold for control update

TARGET_SPEED = 10.0 / 3.6  # [m/s]
N_IND_SEARCH = 10          # Nearest-index search window
MAX_TIME = 500.0           # Maximum simulation time [s]

# Cost matrices
R = np.diag([0.01, 0.01])
Rd = np.diag([0.01, 1.0])
Q = np.diag([1.0, 1.0, 0.5, 0.5])
Qf = Q

GOAL_DIS = 1.5           # Goal distance tolerance [m]
STOP_SPEED = 0.5 / 3.6   # Stop speed threshold [m/s]

# ==== Robot Parameters ====
WHEEL_BASE = 0.2              # [m]
MAX_STEER = 0.523598767       # [rad]
WB = WHEEL_BASE

# ==== Helper Functions and Classes ====
def pi_2_pi(angle):
    return angle_mod(angle)

class State:
    """
    Vehicle state class.
    """
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = 0.0

def update_state(state, a, delta):
    """
    Update vehicle state given acceleration and steering input.
    """
    # Clamp steering angle
    if delta > MAX_STEER:
        delta = MAX_STEER
    elif delta < -MAX_STEER:
        delta = -MAX_STEER

    state.x += state.v * math.cos(state.yaw) * DT
    state.y += state.v * math.sin(state.yaw) * DT
    state.yaw += state.v / WB * math.tan(delta) * DT
    state.v += a * DT

    # Limit speed to target (for this example)
    if state.v > TARGET_SPEED:
        state.v = TARGET_SPEED
    if state.v < 0.0:
        state.v = 0.0

    return state

def get_linear_model_matrix(v, phi, delta):
    A = np.eye(NX)
    A[0, 2] = DT * math.cos(phi)
    A[0, 3] = -DT * v * math.sin(phi)
    A[1, 2] = DT * math.sin(phi)
    A[1, 3] = DT * v * math.cos(phi)
    A[3, 2] = DT * math.tan(delta) / WB

    B = np.zeros((NX, NU))
    B[2, 0] = DT
    B[3, 1] = DT * v / (WB * (math.cos(delta) ** 2))

    C = np.zeros(NX)
    C[0] = DT * v * math.sin(phi) * phi
    C[1] = -DT * v * math.cos(phi) * phi
    C[3] = -DT * v * delta / (WB * (math.cos(delta) ** 2))  # Corrected indexing

    return A, B, C


def get_nparray_from_matrix(x):
    return np.array(x).flatten()

def predict_motion(x0, oa, od, xref):
    """
    Predict the state trajectory using the current control sequence.
    """
    xbar = np.zeros((NX, T + 1))
    xbar[:, 0] = x0
    state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
    for i in range(T):
        state = update_state(state, oa[i], od[i])
        xbar[0, i+1] = state.x
        xbar[1, i+1] = state.y
        xbar[2, i+1] = state.v
        xbar[3, i+1] = state.yaw
    return xbar

def linear_mpc_control(xref, xbar, x0, dref):
    """
    Solve the linear MPC problem.
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

        if t < (T - 1):
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
            constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <= MAX_STEER * DT]

    cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)
    constraints += [x[:, 0] == x0]
    constraints += [x[2, :] <= TARGET_SPEED]
    constraints += [x[2, :] >= 0.0]
    constraints += [cvxpy.abs(u[0, :]) <= 1.0]  # acceleration limit (example)
    constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.CVXOPT, verbose=False)

    if prob.status in [cvxpy.OPTIMAL, cvxpy.OPTIMAL_INACCURATE]:
        ox = get_nparray_from_matrix(x.value[0, :])
        oy = get_nparray_from_matrix(x.value[1, :])
        ov = get_nparray_from_matrix(x.value[2, :])
        oyaw = get_nparray_from_matrix(x.value[3, :])
        oa = get_nparray_from_matrix(u.value[0, :])
        odelta = get_nparray_from_matrix(u.value[1, :])
    else:
        print("Error: MPC problem not solved!")
        ox, oy, ov, oyaw, oa, odelta = None, None, None, None, None, None

    return oa, odelta, ox, oy, oyaw, ov

def iterative_linear_mpc_control(xref, x0, dref, oa, od):
    """
    Iterative MPC control with updating of the operating point.
    """
    if oa is None or od is None:
        oa = [0.0] * T
        od = [0.0] * T

    for _ in range(MAX_ITER):
        xbar = predict_motion(x0, oa, od, xref)
        poa, pod = oa.copy(), od.copy()
        oa, od, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, dref)
        du = sum(abs(np.array(oa) - np.array(poa))) + sum(abs(np.array(od) - np.array(pod)))
        if du <= DU_TH:
            break
    else:
        print("Warning: Max iterative MPC reached")
    return oa, od, ox, oy, oyaw, ov

def calc_ref_trajectory(state, cx, cy, cyaw, sp, dl, pind):
    """
    Calculate the reference trajectory for the MPC horizon.
    """
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))
    ncourse = len(cx)
    # Nearest index search
    dx = [state.x - cx[i] for i in range(pind, min(pind + N_IND_SEARCH, ncourse))]
    dy = [state.y - cy[i] for i in range(pind, min(pind + N_IND_SEARCH, ncourse))]
    d = [dx_i ** 2 + dy_i ** 2 for dx_i, dy_i in zip(dx, dy)]
    ind_offset = np.argmin(d)
    ind = pind + ind_offset

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp[ind]
    xref[3, 0] = cyaw[ind]
    dref[0, 0] = 0.0

    travel = 0.0
    for i in range(T + 1):
        travel += state.v * DT
        dind = int(round(travel / dl))
        if ind + dind < ncourse:
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

def check_goal(state, goal, tind, nind):
    """
    Check if the goal is reached.
    """
    dx = state.x - goal[0]
    dy = state.y - goal[1]
    d = math.hypot(dx, dy)
    if d <= GOAL_DIS and abs(state.v) <= STOP_SPEED and abs(tind - nind) < 5:
        return True
    return False

def get_course_from_waypoints(waypoints, dl=0.5):
    """
    Generate a smooth course using cubic spline from loaded waypoints.
    Here, we assume waypoints is an array of (x, y) positions.
    """
    ax = waypoints[:, 0].tolist()
    ay = waypoints[:, 1].tolist()
    cx, cy, cyaw, ck, _ = cubic_spline_planner.calc_spline_course(ax, ay, ds=dl)
    return cx, cy, cyaw, ck

# ==== ROS2 Node ====
class MPCNode(Node):
    def __init__(self):
        super().__init__('mpc_node')
        self.get_logger().info("MPC Node started")
        # Publishers for steering and wheel speed commands
        self.pub_steering = self.create_publisher(
            Float64MultiArray,
            '/position_controllers/commands',
            10
        )
        self.pub_wheel_spd = self.create_publisher(
            Float64MultiArray,
            '/velocity_controllers/commands',
            10
        )
        # Subscriber for Gazebo model states (odometry)
        self.subscription_gazebo = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )
        # Load waypoints from a YAML file in the "path_tracking" package
        path_tracking_package = get_package_share_directory("path_tracking")
        wp = self.load_waypoints(f'{path_tracking_package}/path_data/path.yaml')
        wp = np.array(wp)
        self.waypoints = wp[:, 0:2]
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints.")
        # Generate a smooth course from waypoints
        self.cx, self.cy, self.cyaw, self.ck = get_course_from_waypoints(self.waypoints, dl=0.5)
        self.sp = [TARGET_SPEED] * len(self.cx)
        # State variables
        self.current_state = None
        self.target_ind = 0
        self.oa = None
        self.od = None
        self.t_sim = 0.0
        # Timer for MPC control loop (adjust period as needed)
        self.control_timer = self.create_timer(0.01, self.control_callback)

    def load_waypoints(self, file_path):
        """Load waypoints from a YAML file."""
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
        # Each waypoint is expected to have 'x', 'y', and optionally 'yaw'
        waypoints = [(wp['x'], wp['y'], wp.get('yaw', 0.0)) for wp in data]
        return waypoints

    def model_states_callback(self, msg):
        """
        Callback for /gazebo/model_states.
        Here we assume the robot model is the first in the list.
        Adjust the index or use model name matching as needed.
        """
        try:
            index = 0  # Modify if needed
            # Extract velocity magnitude from twist information
            v = math.sqrt(
                msg.twist[index].linear.x**2 +
                msg.twist[index].linear.y**2 +
                msg.twist[index].linear.z**2
            )
            self.current_state = State(
                x=msg.pose[index].position.x,
                y=msg.pose[index].position.y,
                yaw=self.quaternion_to_yaw(msg.pose[index].orientation),
                v=v
            )
        except Exception as e:
            self.get_logger().error(f"Error in model_states_callback: {e}")


    def quaternion_to_yaw(self, q):
        """
        Convert quaternion to yaw angle.
        """
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def publish_steering(self, steering_L: float, steering_R: float):
        steering_msg = Float64MultiArray()
        steering_msg.data = [steering_L, steering_R]
        self.pub_steering.publish(steering_msg)

    def publish_wheel_speed(self, wheelspeed_L: float, wheelspeed_R: float):
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [wheelspeed_L, wheelspeed_R]
        self.pub_wheel_spd.publish(wheel_msg)


    def control_callback(self):
        """
        Periodic callback to compute MPC and publish control commands.
        """
        if self.current_state is None:
            return

        # Run MPC using current state and reference course
        state = self.current_state
        x0 = [state.x, state.y, state.v, state.yaw]
        xref, self.target_ind, dref = calc_ref_trajectory(
            state, self.cx, self.cy, self.cyaw, self.sp, dl=0.5, pind=self.target_ind
        )
        self.oa, self.od, ox, oy, oyaw, ov = iterative_linear_mpc_control(
            xref, x0, dref, self.oa, self.od
        )
        if self.od is not None:
            # Use the first computed control input
            steering_cmd = self.od[0]
            accel_cmd = self.oa[0]
            # Update internal state with MPC output (for simulation purposes)
            self.current_state = update_state(state, accel_cmd, steering_cmd)
            self.t_sim += DT

            # Publish commands using Float64MultiArray

            self.publish_steering(steering_cmd, steering_cmd)

            # Here, we assume wheel speed is proportional to desired velocity.
            # This mapping may require adjustment for your robot.
            wheel_speed = self.current_state.v
            self.publish_wheel_speed(wheel_speed, wheel_speed)

            self.get_logger().info(
                f"t:{self.t_sim:.1f} x:{state.x:.2f} y:{state.y:.2f} v:{state.v:.2f} "
                f"steer:{steering_cmd:.3f} accel:{accel_cmd:.3f}"
            )

            # Optionally, check if the goal is reached
            goal = [self.cx[-1], self.cy[-1]]
            if check_goal(state, goal, self.target_ind, len(self.cx)):
                self.get_logger().info("Goal reached!")
                # Optionally, shut down or reset the controller

def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

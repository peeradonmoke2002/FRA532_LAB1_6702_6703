#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
import yaml
import numpy as np
import math
import casadi as ca
import matplotlib.pyplot as plt

# MPC parameters
DT = 0.15      # Time step [s]
N = 10         # Horizon length (timesteps in horizon)
WB = 0.2       # Wheelbase of the robot [m]

# MANUAL MPC PARAMETERS (Using Optimized Values)
Q = np.diag([10.0, 10.0, 6.0, 2.0])   # State error penalty
Qf = Q                                # Terminal cost
R = np.diag([3.0, 1.73436065])         # Control effort penalty
Rd = np.diag([0.2, 30.1])              # Input smoothness penalty

class MPCPathFollower(Node):
    def __init__(self):
        super().__init__('mpc_path_follower')
        self.create_subscription(ModelStates, '/gazebo/model_states', self.gazebo_callback, 10)
        self.waypoints = self.load_path('/home/tang/ros2_lab1_m/src/FRA532_LAB1_6702_6703/robot_controller/data/path.yaml')
        self.get_logger().info("Path loaded successfully.")
        self.pub_steering = self.create_publisher(JointTrajectory, '/joint_trajectory_position_controller/joint_trajectory', 10)
        self.pub_wheel_spd = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)
        
        # Robot state
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.v = 0.0
        self.target_speed = 25  # [m/s]
        
        # Maximum steering angle (radians)
        self.max_steering_angle = 0.5

        # For plotting (optional)
        self.robot_x = []
        self.robot_y = []
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8,6))
    
    def load_path(self, filename):
        """
        Load the path from a YAML file.
        """
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
        if 'path' not in data:
            self.get_logger().error(f"Key 'path' not found in {filename}")
            return []
        return np.array([(point['x'], point['y']) for point in data['path']])
    
    def should_skip_waypoint(self, current_pos, waypoint):
        """
        Returns True if the waypoint is too far or requires an excessive yaw change.
        """
        wp_x, wp_y = waypoint
        distance = np.linalg.norm(np.array([self.x, self.y]) - np.array([wp_x, wp_y]))
        
        yaw_to_wp = math.atan2(wp_y - self.y, wp_x - self.x)
        yaw_diff = self.normalize_angle(yaw_to_wp - self.yaw)

        # Skip waypoint if it is too far or requires a large yaw change
        if distance > 3.0 or abs(yaw_diff) > np.deg2rad(75):
            return True
        return False
    
    def normalize_angle(self, angle):
        """
        Normalize angle to be within [-pi, pi].
        """
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def get_reference_trajectory(self, current_pos, horizon, min_distance=0.2):
        """
        Generate a reference trajectory of waypoints for the given horizon.
        """
        ref_traj = []
        
        for i, wp in enumerate(self.waypoints):
            wp_x, wp_y = wp[0], wp[1]

            # Compute yaw if not provided
            if len(wp) > 2:
                wp_yaw = wp[2]
            else:
                wp_yaw = math.atan2(wp_y - current_pos[1], wp_x - current_pos[0])

            vector = np.array([wp_x, wp_y]) - np.array(current_pos)
            angle_to_wp = math.atan2(vector[1], vector[0])
            angle_diff = self.normalize_angle(angle_to_wp - self.yaw)

            max_angle_threshold = np.deg2rad(60)  # Allow ±60° in front
            if abs(angle_diff) > max_angle_threshold:
                continue

            # Skip very close waypoints
            distance_to_wp = np.linalg.norm(np.array([self.x, self.y]) - np.array([wp_x, wp_y]))
            if distance_to_wp < min_distance:
                continue

            # Skip waypoint if unreachable
            if self.should_skip_waypoint(current_pos, [wp_x, wp_y]):
                continue  

            ref_traj.append([wp_x, wp_y, wp_yaw])
            if len(ref_traj) >= horizon:
                break

        # If not enough waypoints, repeat the last one
        if len(ref_traj) < horizon:
            last_wp = self.waypoints[-1]
            last_wp_yaw = last_wp[2] if len(last_wp) > 2 else 0.0
            ref_traj.extend([[last_wp[0], last_wp[1], last_wp_yaw]] * (horizon - len(ref_traj)))

        if ref_traj:
            next_wp_x, next_wp_y, next_wp_yaw = ref_traj[0]
            self.get_logger().info(f"Next Waypoint: x={next_wp_x:.3f}, y={next_wp_y:.3f}, yaw={math.degrees(next_wp_yaw):.2f}°")

        return np.array(ref_traj)

    def mpc_control(self):
        """
        Compute control commands via MPC using iterative linearization.
        Model state: [x, y, yaw, v]
        Control inputs: [a, delta]
        """
        current_pos = [self.x, self.y]
        # 1) Create reference trajectory (N+1 steps in horizon)
        ref_traj = self.get_reference_trajectory(current_pos, horizon=N+1, min_distance=0.1)

        # 2) Compute reference yaw from the waypoints
        ref_yaw = []
        for j in range(len(ref_traj)-1):
            dy = ref_traj[j+1, 1] - ref_traj[j, 1]
            dx = ref_traj[j+1, 0] - ref_traj[j, 0]
            ref_yaw.append(math.atan2(dy, dx))
        if ref_yaw:
            ref_yaw.append(ref_yaw[-1])
        else:
            ref_yaw.append(self.yaw)
        ref_yaw = np.array(ref_yaw)

        # 3) Set reference speed
        ref_speed = self.target_speed

        # 4) Estimate initial values for velocity and steering
        v_nom = np.full(N, self.v)
        u_nom = np.zeros(N)  # Nominal steering input

        # Nominal state trajectory for linearization
        x_nom = np.full(N+1, self.x)
        y_nom = np.full(N+1, self.y)
        yaw_nom = np.full(N+1, self.yaw)

        max_iter = 5   # Maximum number of iterations for convergence
        tol = 1e-3     # Convergence tolerance

        sol = None  # To store the solution
        for it in range(max_iter):
            opti = ca.Opti()

            # Decision variables
            x_var = opti.variable(N+1)
            y_var = opti.variable(N+1)
            yaw_var = opti.variable(N+1)
            v_var = opti.variable(N+1)
            a_var = opti.variable(N)       # Acceleration
            delta_var = opti.variable(N)   # Steering angle

            cost = 0

            # Initial condition constraints
            opti.subject_to(x_var[0] == self.x)
            opti.subject_to(y_var[0] == self.y)
            opti.subject_to(yaw_var[0] == self.yaw)
            opti.subject_to(v_var[0] == self.v)

            for k in range(N):
                # State error cost
                state_error = ca.vertcat(
                    x_var[k] - ref_traj[k, 0],
                    y_var[k] - ref_traj[k, 1],
                    yaw_var[k] - ref_yaw[k],
                    v_var[k] - ref_speed
                )
                cost += ca.mtimes([state_error.T, state_error])
                
                # Control effort cost
                u_vec = ca.vertcat(a_var[k], delta_var[k])
                cost += ca.mtimes([u_vec.T, R, u_vec])
                
                # Input smoothness cost
                if k > 0:
                    input_diff = ca.vertcat(a_var[k] - a_var[k-1], delta_var[k] - delta_var[k-1])
                    cost += ca.mtimes([input_diff.T, Rd, input_diff])
                
                # Soft penalty for exceeding steering angle limits
                big_lambda = 1215000.0
                penalty = big_lambda * (ca.fmax(0, ca.fabs(delta_var[k]) - self.max_steering_angle)**2)
                cost += penalty

                # Linearized dynamics: use nominal state of current timestep
                if it == 0:
                    yaw_nom_k = self.yaw
                else:
                    yaw_nom_k = yaw_nom[k]
                v_nom_k = v_nom[k]
                delta_nom = u_nom[k]

                cos_yaw = math.cos(yaw_nom_k)
                sin_yaw = math.sin(yaw_nom_k)

                # Linearized dynamics for x
                linearized_x = x_var[k] + DT * (
                    v_nom_k * cos_yaw +
                    cos_yaw * (v_var[k] - v_nom_k) -
                    v_nom_k * sin_yaw * (yaw_var[k] - yaw_nom_k)
                )

                # Linearized dynamics for y
                linearized_y = y_var[k] + DT * (
                    v_nom_k * sin_yaw +
                    sin_yaw * (v_var[k] - v_nom_k) +
                    v_nom_k * cos_yaw * (yaw_var[k] - yaw_nom_k)
                )

                # Linearized dynamics for yaw (using bilinear expansion)
                tan_nom = math.tan(delta_nom) if abs(delta_nom) < 1.4 else math.tan(1.4)
                bilinear_term = (
                    v_nom_k * tan_nom +
                    tan_nom * (v_var[k] - v_nom_k) +
                    v_nom_k * (1.0 / (math.cos(delta_nom) ** 2)) * (delta_var[k] - delta_nom)
                )
                linearized_yaw = yaw_var[k] + (DT / WB) * bilinear_term

                # Linearized dynamics for velocity: v_{k+1} = v_k + DT * a_k
                linearized_v = v_var[k] + DT * a_var[k]

                # Add dynamics constraints for next state
                opti.subject_to(x_var[k+1] == linearized_x)
                opti.subject_to(y_var[k+1] == linearized_y)
                opti.subject_to(yaw_var[k+1] == linearized_yaw)
                opti.subject_to(v_var[k+1] == linearized_v)

                # Input constraints
                opti.subject_to(ca.fabs(delta_var[k]) <= self.max_steering_angle)
                opti.subject_to(a_var[k] <= 1.0)
                opti.subject_to(a_var[k] >= -1.0)

            # Terminal cost
            terminal_error = ca.vertcat(
                x_var[N] - ref_traj[N, 0],
                y_var[N] - ref_traj[N, 1],
                yaw_var[N] - ref_yaw[N],
                v_var[N] - ref_speed
            )
            cost += ca.mtimes([terminal_error.T, Qf, terminal_error])

            opti.minimize(cost)

            # Set solver options (using IPOPT)
            p_opts = {"print_time": False}
            s_opts = {"print_level": 0}
            opti.solver("ipopt", p_opts, s_opts)
            try:
                sol = opti.solve()
            except RuntimeError as e:
                self.get_logger().error("MPC solve error with CasADi: " + str(e))
                return 0.0, 0.0

            # Retrieve solution values
            v_nom_new = np.array([sol.value(v_var[k]) for k in range(N)])
            delta_nom_new = np.array([sol.value(delta_var[k]) for k in range(N)])

            # Simulate nominal state trajectory using the current inputs
            x_nom[0] = self.x
            y_nom[0] = self.y
            yaw_nom[0] = self.yaw
            for k in range(N):
                cos_yaw_nom = math.cos(yaw_nom[k])
                sin_yaw_nom = math.sin(yaw_nom[k])
                x_nom[k+1] = x_nom[k] + DT * (v_nom[k] * cos_yaw_nom)
                y_nom[k+1] = y_nom[k] + DT * (v_nom[k] * sin_yaw_nom)
                delta_nom_k = u_nom[k]
                tan_val = math.tan(delta_nom_k) if abs(delta_nom_k) < 1.4 else math.tan(1.4)
                yaw_nom[k+1] = yaw_nom[k] + (DT / WB) * (v_nom[k] * tan_val)

            # Check if nominal inputs have converged
            if (np.linalg.norm(v_nom_new - v_nom) < tol) and (np.linalg.norm(delta_nom_new - u_nom) < tol):
                v_nom = v_nom_new
                u_nom = delta_nom_new
                break

            # Update nominal values for the next iteration 
            v_nom = v_nom_new
            u_nom = delta_nom_new

        # Use the first control command from the solution
        a_cmd = sol.value(a_var[0])
        delta_cmd = sol.value(delta_var[0])
        delta_cmd = np.clip(delta_cmd, -self.max_steering_angle, self.max_steering_angle)

        return a_cmd, delta_cmd

    def search_nearest_index(self):
        """
        Calculate the Euclidean distance from the current position to all waypoints,
        and return the index of the nearest waypoint.
        """
        distances = np.linalg.norm(self.waypoints - np.array([self.x, self.y]), axis=1)
        return np.argmin(distances)

    def gazebo_callback(self, msg):
        """
        Callback function for processing incoming Gazebo model states.
        """
        try:
            index = msg.name.index("limo")
        except ValueError:
            self.get_logger().error("Robot model not found in Gazebo!")
            return
        
        pose = msg.pose[index]
        self.x = pose.position.x
        self.y = pose.position.y
        self.yaw = self.quaternion_to_euler(pose.orientation)
        
        # Display distance to the next waypoint
        idx = self.search_nearest_index()
        dist = np.hypot(self.waypoints[idx, 0] - self.x, self.waypoints[idx, 1] - self.y)
        self.get_logger().info(f"Distance to next waypoint: {dist:.3f} m")
        
        acceleration, steering_cmd = self.mpc_control()
        
        # Update robot velocity
        self.v += acceleration * DT
        self.v = min(self.v, self.target_speed)
        
        self.publish_steering(steering_cmd)
        self.publish_wheel_speed(self.v)
        
        self.get_logger().info(
            f"Pose: x={self.x:.3f}, y={self.y:.3f}, yaw={math.degrees(self.yaw):.2f}° | "
            f"Steering: {math.degrees(steering_cmd):.2f}° | Speed: {self.v:.2f} m/s"
        )
        
        # Update plotting data
        self.robot_x.append(self.x)
        self.robot_y.append(self.y)
        self.update_plot()

    def quaternion_to_euler(self, q):
        """
        Convert quaternion to Euler yaw angle.
        """
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def publish_steering(self, steering):
        """
        Publish the steering command as a JointTrajectory message.
        """
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['front_left_steering', 'front_right_steering']
        point = JointTrajectoryPoint()
        point.positions = [steering, steering]
        traj_msg.points.append(point)
        self.pub_steering.publish(traj_msg)

    def publish_wheel_speed(self, speed):
        """
        Publish the wheel speed command as a Float64MultiArray message.
        """
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [speed, speed]
        self.pub_wheel_spd.publish(wheel_msg)

    def update_plot(self):
        """
        Update the live plot (if enabled). Currently commented out.
        """
        # self.ax.clear()
        # self.ax.plot(self.waypoints[:, 0], self.waypoints[:, 1], 'go-', label="Planned Path")
        # self.ax.plot(self.robot_x, self.robot_y, 'r.-', label="Actual Path")
        # self.ax.scatter(self.robot_x[-1], self.robot_y[-1], c='purple', marker='x', label="Current Position")
        # self.ax.set_title("MPC Controller")
        # self.ax.legend()
        # plt.draw()
        # plt.pause(0.1)
        pass

def main():
    rclpy.init()
    node = MPCPathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

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
import os

# MPC parameters
DT = 0.15      # time step [s]
N = 20         # horizon length (timesteps in horizon)
WB = 0.2       # [m] wheelbase of the robot

# MPC Cost Matrices
Q = np.diag([0.5, 0.5, 0.1, 0.5])   # State error penalty
Qf = Q                                 # Terminal cost
R = np.diag([1.0, 1.0])          # Control effort penalty
Rd = np.diag([0.2, 0.1])               # Smooth control transitions

class MPCPathFollower(Node):
    def __init__(self):
        super().__init__('mpc_path_follower')
        self.create_subscription(ModelStates, '/gazebo/model_states', self.gazebo_callback, 10)
        self.waypoints = self.load_path("path.yaml")
        self.get_logger().info("✅ Path loaded successfully.")
        
        # New position control: publish steering commands as Float64MultiArray.
        self.pub_steering = self.create_publisher(
            Float64MultiArray,
            "/position_controllers/commands",
            10
        )
        self.pub_wheel_spd = self.create_publisher(
            Float64MultiArray, 
            '/velocity_controllers/commands', 
            10
        )
        
        # Robot state variables
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.v = 0.0
        self.target_speed = 6  # [m/s]
        self.max_steering_angle = 0.5

        self.robot_x = []
        self.robot_y = []
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8,6))
    
    def load_path(self, filename):
        if not os.path.isabs(filename):
            ros_workspace = os.getenv("ROS_WORKSPACE")
            if ros_workspace is None:
                script_dir = os.path.dirname(os.path.realpath(__file__))
                ros_workspace = script_dir.split('/src/')[0]
            filename = os.path.join(ros_workspace, "src", "FRA532_LAB1_6702_6703", "path_tracking", "data", filename)
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
        return np.array([(point['x'], point['y'], point.get('yaw', 0.0)) for point in data['path']])
    
    def normalize_angle(self, angle):
        """
        Normalize an angle to the range [-pi, pi].
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def get_reference_trajectory(self, current_pos, horizon, min_distance=0.1):
        ref_traj = []
        for wp in self.waypoints:
            wp_x, wp_y = wp[0], wp[1]
            # Use provided yaw if available, else compute from current position.
            if len(wp) > 2:
                wp_yaw = wp[2]
            else:
                wp_yaw = math.atan2(wp_y - current_pos[1], wp_x - current_pos[0])
            distance_to_wp = np.linalg.norm(np.array([self.x, self.y]) - np.array([wp_x, wp_y]))
            if distance_to_wp < min_distance:
                continue
            ref_traj.append([wp_x, wp_y, wp_yaw])
            if len(ref_traj) >= horizon:
                break
        if len(ref_traj) < horizon:
            last_wp = self.waypoints[-1]
            last_wp_yaw = last_wp[2] if len(last_wp) > 2 else 0.0
            ref_traj.extend([[last_wp[0], last_wp[1], last_wp_yaw]] * (horizon - len(ref_traj)))
        if ref_traj:
            next_wp = ref_traj[0]
            self.get_logger().info(f"🎯 Next Waypoint: x={next_wp[0]:.3f}, y={next_wp[1]:.3f}, θ={math.degrees(next_wp[2]):.2f}°")
        return np.array(ref_traj)

    def mpc_control(self):
        current_pos = [self.x, self.y]
        ref_traj = self.get_reference_trajectory(current_pos, horizon=N+1, min_distance=0.1)

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
        ref_speed = self.target_speed

        # Nominal values (for linearization)
        v_nom = np.full(N, self.v)
        u_nom = np.zeros(N)
        x_nom = np.full(N+1, self.x)
        y_nom = np.full(N+1, self.y)
        yaw_nom = np.full(N+1, self.yaw)
        


        max_iter = 5
        tol = 1e-3
        sol = None
        for it in range(max_iter):
            opti = ca.Opti()
            x_var   = opti.variable(N+1)
            y_var   = opti.variable(N+1)
            yaw_var = opti.variable(N+1)
            v_var   = opti.variable(N+1)
            a_var      = opti.variable(N)
            delta_var  = opti.variable(N)

            cost = 0
            opti.subject_to(x_var[0] == self.x)
            opti.subject_to(y_var[0] == self.y)
            opti.subject_to(yaw_var[0] == self.yaw)
            opti.subject_to(v_var[0] == self.v)

            for k in range(N):
                state_error = ca.vertcat(
                    x_var[k] - ref_traj[k, 0],
                    y_var[k] - ref_traj[k, 1],
                    yaw_var[k] - ref_yaw[k],
                    v_var[k] - ref_speed
                )
                cost += ca.mtimes([state_error.T, state_error])
                u_vec = ca.vertcat(a_var[k], delta_var[k])
                cost += ca.mtimes([u_vec.T, R, u_vec])
                if k > 0:
                    input_diff = ca.vertcat(a_var[k] - a_var[k-1], delta_var[k] - delta_var[k-1])
                    cost += ca.mtimes([input_diff.T, Rd, input_diff])
                big_lambda = 1215000.0
                penalty = big_lambda * (ca.fmax(0, ca.fabs(delta_var[k]) - self.max_steering_angle)**2)
                cost += penalty

                if it == 0:
                    yaw_nom_k = self.yaw
                else:
                    yaw_nom_k = yaw_nom[k]
                v_nom_k = v_nom[k]
                delta_nom = u_nom[k]
                cos_yaw = math.cos(yaw_nom_k)
                sin_yaw = math.sin(yaw_nom_k)

                linearized_x = x_var[k] + DT * (
                    v_nom_k * cos_yaw +
                    cos_yaw * (v_var[k] - v_nom_k) -
                    v_nom_k * sin_yaw * (yaw_var[k] - yaw_nom_k)
                )
                linearized_y = y_var[k] + DT * (
                    v_nom_k * sin_yaw +
                    sin_yaw * (v_var[k] - v_nom_k) +
                    v_nom_k * cos_yaw * (yaw_var[k] - yaw_nom_k)
                )
                tan_nom = math.tan(delta_nom) if abs(delta_nom) < 1.4 else math.tan(1.4)
                bilinear_term = (
                    v_nom_k * tan_nom +
                    tan_nom * (v_var[k] - v_nom_k) +
                    v_nom_k * (1.0 / (math.cos(delta_nom) ** 2)) * (delta_var[k] - delta_nom)
                )
                linearized_yaw = yaw_var[k] + (DT / WB) * bilinear_term
                linearized_v = v_var[k] + DT * a_var[k]
                
                opti.subject_to(x_var[k+1] == linearized_x)
                opti.subject_to(y_var[k+1] == linearized_y)
                opti.subject_to(yaw_var[k+1] == linearized_yaw)
                opti.subject_to(v_var[k+1] == linearized_v)
                opti.subject_to(ca.fabs(delta_var[k]) <= self.max_steering_angle)
                opti.subject_to(a_var[k] <= 1.0)
                opti.subject_to(a_var[k] >= -1.0)

            terminal_error = ca.vertcat(
                x_var[N] - ref_traj[N, 0],
                y_var[N] - ref_traj[N, 1],
                yaw_var[N] - ref_yaw[N],
                v_var[N] - ref_speed
            )
            cost += ca.mtimes([terminal_error.T, Qf, terminal_error])
            opti.minimize(cost)

            p_opts = {"print_time": False}
            s_opts = {"print_level": 0}
            opti.solver("ipopt", p_opts, s_opts)
            try:
                sol = opti.solve()
            except RuntimeError as e:
                self.get_logger().error("❌ MPC solve error with CasADi: " + str(e))
                return 0.0, 0.0

            v_nom_new = np.array([sol.value(v_var[k]) for k in range(N)])
            delta_nom_new = np.array([sol.value(delta_var[k]) for k in range(N)])
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
            if (np.linalg.norm(v_nom_new - v_nom) < tol) and (np.linalg.norm(delta_nom_new - u_nom) < tol):
                v_nom = v_nom_new
                u_nom = delta_nom_new
                break
            v_nom = v_nom_new
            u_nom = delta_nom_new

        a_cmd = sol.value(a_var[0])
        delta_cmd = sol.value(delta_var[0])
        delta_cmd = np.clip(delta_cmd, -self.max_steering_angle, self.max_steering_angle)
        return a_cmd, delta_cmd

    def search_nearest_index(self):
        distances = np.linalg.norm(self.waypoints[:, :2] - np.array([self.x, self.y]), axis=1)
        return np.argmin(distances)

    def gazebo_callback(self, msg):
        try:
            index = msg.name.index("limo")
        except ValueError:
            self.get_logger().error("Robot model not found in Gazebo!")
            return
        
        pose = msg.pose[index]
        self.x = pose.position.x
        self.y = pose.position.y
        self.yaw = self.euler_from_quaternion(pose.orientation)
        
        idx = self.search_nearest_index()
        dist = np.hypot(self.waypoints[idx, 0] - self.x, self.waypoints[idx, 1] - self.y)
        self.get_logger().info(f"Distance to next waypoint: {dist:.3f} m")
        
        acceleration, steering_cmd = self.mpc_control()
        self.v += acceleration * DT
        self.v = min(self.v, self.target_speed)
        
        # Use new position control: publish the same steering command for both steering joints.
        self.publish_steering(steering_cmd)
        self.publish_wheel_speed(self.v)
        
        self.get_logger().info(
            f"📍 Pose: x={self.x:.3f}, y={self.y:.3f}, yaw={math.degrees(self.yaw):.2f}° | "
            f"🔄 Steering: {math.degrees(steering_cmd):.2f}° | 🚀 Speed: {self.v:.2f} m/s"
        )
        
        self.robot_x.append(self.x)
        self.robot_y.append(self.y)
        self.update_plot()

    def euler_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return yaw

    def publish_steering(self, steering: float):
        # New position control: publish steering commands as Float64MultiArray.
        # We send the same command for both front steering joints.
        steering_msg = Float64MultiArray()
        steering_msg.data = [steering, steering]
        self.pub_steering.publish(steering_msg)
        self.get_logger().info(f"🔄 Steering Published (Float64MultiArray): {steering:.3f} rad")

    def publish_wheel_speed(self, speed):
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [speed, speed]
        self.pub_wheel_spd.publish(wheel_msg)
        self.get_logger().info(f"⚡ Wheel Speed Published: {speed:.3f} m/s")

    def update_plot(self):
        # Update plotting if needed.
        pass

def main():
    rclpy.init()
    node = MPCPathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

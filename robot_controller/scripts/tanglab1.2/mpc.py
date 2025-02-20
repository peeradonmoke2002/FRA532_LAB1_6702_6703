#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
import yaml
import numpy as np
import math
import cvxpy as cp
import matplotlib.pyplot as plt

# MPC parameters
DT = 0.15      # time step [s]
N = 3         # horizon length ( timesteps in horizon)
WB = 0.2      # [m] wheelbase of the robot

# Cost matrices
Q = np.diag([5.0, 5.0, 0.1, 5.0])   # penalize error in [x, y, yaw, v]
Qf = Q                              # terminal state cost
R = np.diag([0.1, 0.2])             # penalize input effort [acceleration, steering]
Rd = np.diag([0.51, 0.51])          # penalize input difference

class MPCPathFollower(Node):
    def __init__(self):
        super().__init__('mpc_path_follower')
        self.create_subscription(ModelStates, '/gazebo/model_states', self.gazebo_callback, 10)
        
        # Load waypoints from YAML
        self.waypoints = self.load_path('/home/tang/ros2_lab1_m/src/FRA532_LAB1_6702_6703/robot_controller/data/path.yaml')
        self.get_logger().info("✅ Path loaded successfully.")
        
        # ROS2 Publishers
        self.pub_steering = self.create_publisher(JointTrajectory, '/joint_trajectory_position_controller/joint_trajectory', 10)
        self.pub_wheel_spd = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)
        
        # Robot state
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.v = 0.0
        self.target_speed = 4.5  # [m/s]
        
        # Maximum steering angle (radians)
        self.max_steering_angle = 0.523598767 / 5  # ~15° (can adjust)
        
        # For plotting (optional)
        self.robot_x = []
        self.robot_y = []
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8,6))
        
    def load_path(self, filename):
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
        if 'path' not in data:
            self.get_logger().error(f"⚠️ Key 'path' not found in {filename}")
            return []
        return np.array([(point['x'], point['y']) for point in data['path']])
    
    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def get_reference_trajectory(self, current_pos, horizon, min_distance=0.1):
        """
        Select a reference trajectory from waypoints, considering both the heading and angle of the waypoints when it comes to signal reception.
        and display information of the next waypoint
        """
        heading = np.array([math.cos(self.yaw), math.sin(self.yaw)])
        ref_traj = []
        
        for wp in self.waypoints:
            wp_x, wp_y = wp[0], wp[1]
            
            # check info in waypoint is yaw or not
            if len(wp) > 2:
                wp_yaw = wp[2]  # if has yaw in file use YAML 
            else:
                wp_yaw = math.atan2(wp_y - current_pos[1], wp_x - current_pos[0])  #  x, y

            vector = np.array([wp_x, wp_y]) - np.array(current_pos)

            # angle waypoint
            angle_to_wp = math.atan2(vector[1], vector[0])
            angle_diff = self.normalize_angle(angle_to_wp - self.yaw)

            max_angle_threshold = np.deg2rad(60)  #  Let waypoints +/-60° count as "front
            if abs(angle_diff) > max_angle_threshold:
                continue

           # Check if it is far from the last waypoint in the trajectory.
            if ref_traj:
                if np.linalg.norm(np.array([wp_x, wp_y]) - np.array(ref_traj[-1][:2])) < min_distance:
                    continue

            ref_traj.append([wp_x, wp_y, wp_yaw])
            if len(ref_traj) >= horizon:
                break

        # If the number of waypoints is not enough, add the last waypoint.
        if len(ref_traj) < horizon:
            last_wp = self.waypoints[-1]
            last_wp_yaw = last_wp[2] if len(last_wp) > 2 else 0.0  # If not, use 0 as default
            ref_traj.extend([[last_wp[0], last_wp[1], last_wp_yaw]] * (horizon - len(ref_traj)))

        # show next waypoint
        if ref_traj:
            next_wp_x, next_wp_y, next_wp_yaw = ref_traj[0]
            self.get_logger().info(f"🎯 Next Waypoint: x={next_wp_x:.3f}, y={next_wp_y:.3f}, θ={math.degrees(next_wp_yaw):.2f}°")

        return np.array(ref_traj)

    def mpc_control(self):
        """
        Calculate input via MPC with iterative linearization.
        Model state: [x, y, yaw, v]
        Control inputs: [a, delta]
        """
        current_pos = [self.x, self.y]
        # create reference trajectory (state references)
        ref_traj = self.get_reference_trajectory(current_pos, horizon=N+1, min_distance=0.1)
        
        # cal reference yaw form trajectory
        ref_yaw = []
        for j in range(len(ref_traj)-1):
            dy = ref_traj[j+1,1] - ref_traj[j,1]
            dx = ref_traj[j+1,0] - ref_traj[j,0]
            ref_yaw.append(math.atan2(dy, dx))
        if ref_yaw:
            ref_yaw.append(ref_yaw[-1])
        else:
            ref_yaw.append(self.yaw)
        ref_yaw = np.array(ref_yaw)
        ref_speed = self.target_speed

        # Set the default nominal values ​​for v and u[1,k]
        v_nom = np.full(N, self.v)     # Suppose current speed is used for every timestep.
        u_nom = np.zeros(N)            # Suppose the steering is initially set to 0.

        max_iter = 5
        tol = 1e-3  # tolerance for check convergence

        for it in range(max_iter):
            # define optimization variables each iteration
            x_var = cp.Variable(N+1)
            y_var = cp.Variable(N+1)
            yaw_var = cp.Variable(N+1)
            v_var = cp.Variable(N+1)
            u = cp.Variable((2, N))  # u[0,:] = acceleration, u[1,:] = steering

            cost = 0
            constraints = []
            
            # initial condition
            constraints += [
                x_var[0] == self.x,
                y_var[0] == self.y,
                yaw_var[0] == self.yaw,
                v_var[0] == self.v
            ]
            
            for k in range(N):
                # cal state error for cost
                state_error = cp.vstack([
                    x_var[k] - ref_traj[k, 0],
                    y_var[k] - ref_traj[k, 1],
                    yaw_var[k] - ref_yaw[k],
                    v_var[k] - ref_speed
                ])
                cost += cp.quad_form(state_error, Q)
                cost += cp.quad_form(u[:, k], R)
                if k > 0:
                    input_diff = u[:, k] - u[:, k-1]
                    cost += cp.quad_form(input_diff, Rd)
                
               
                # Added soft penalty for steering beyond the limit
                lambda_ = 5000.0
                penalty = lambda_ * cp.square(cp.pos(cp.abs(u[1, k]) - self.max_steering_angle))
                cost += penalty
                
                # for dynamics of x,y (still using linearization from yaw currently)
                cos_yaw0 = math.cos(self.yaw)
                sin_yaw0 = math.sin(self.yaw)
                constraints += [
                    x_var[k+1] == x_var[k] + DT * v_var[k] * cos_yaw0,
                    y_var[k+1] == y_var[k] + DT * v_var[k] * sin_yaw0
                ]
                
                # Linearize bilinear term v_var[k] * u[1, k] รอบ (v_nom[k], u_nom[k])
                # use: v_nom*u_nom + u_nom*(v_var[k] - v_nom) + v_nom*(u[1,k] - u_nom)
                linearized_term = v_nom[k]*u_nom[k] + u_nom[k]*(v_var[k] - v_nom[k]) + v_nom[k]*(u[1, k] - u_nom[k])
                constraints += [
                    yaw_var[k+1] == yaw_var[k] + (DT / WB) * linearized_term,
                    v_var[k+1] == v_var[k] + DT * u[0, k]
                ]
                
                # limit input
                constraints += [
                    cp.abs(u[1, k]) <= self.max_steering_angle,
                    u[0, k] <= 1.0,
                    u[0, k] >= -1.0
                ]
            
            # Terminal cost
            terminal_error = cp.vstack([
                x_var[N] - ref_traj[N, 0],
                y_var[N] - ref_traj[N, 1],
                yaw_var[N] - ref_yaw[N],
                v_var[N] - ref_speed
            ])
            cost += cp.quad_form(terminal_error, Qf)
            
            # create optimization
            prob = cp.Problem(cp.Minimize(cost), constraints)
            try:
                prob.solve(solver=cp.OSQP, warm_start=True)
            except Exception as e:
                self.get_logger().error(f"❌ MPC solve error: {e}")
                return 0.0, 0.0
            
            if prob.status not in ["optimal", "optimal_inaccurate"]:
                self.get_logger().warn("⚠️ MPC did not find an optimal solution")
                return 0.0, 0.0
            
            # Retrieve solution values ​​for nominal update
            v_nom_new = np.array([v_var.value[k] for k in range(N)])
            u_nom_new = np.array([u.value[1, k] for k in range(N)])
            
            # check convergence
            if (np.linalg.norm(v_nom_new - v_nom) < tol) and (np.linalg.norm(u_nom_new - u_nom) < tol):
                v_nom = v_nom_new
                u_nom = u_nom_new
                break
            

            #Update nominal values ​​for the next iteration.
            v_nom = v_nom_new
            u_nom = u_nom_new

        # Use the command in the first timestep.
        a_cmd = u.value[0, 0]
        delta_cmd = u.value[1, 0]
        return a_cmd, delta_cmd


    def search_nearest_index(self):
        # Calculates the Euclidean distance between the robot's current position and all waypoints.
        distances = np.linalg.norm(self.waypoints - np.array([self.x, self.y]), axis=1)
        return np.argmin(distances)


    def gazebo_callback(self, msg):
        try:
            index = msg.name.index("limo")
        except ValueError:
            self.get_logger().error("❌ Robot model not found in Gazebo!")
            return
        
        pose = msg.pose[index]
        self.x = pose.position.x
        self.y = pose.position.y
        self.yaw = self.quaternion_to_euler(pose.orientation)
        self.target_speed = 5.5
        
        # Optionally print distance to next waypoint
        dist = np.hypot(self.waypoints[self.search_nearest_index(), 0] - self.x,
                        self.waypoints[self.search_nearest_index(), 1] - self.y)
        self.get_logger().info(f"Distance to next waypoint: {dist:.3f} m")
        
        acceleration, steering_cmd = self.mpc_control()
        
        self.v += acceleration * DT
        self.v = min(self.v, self.target_speed)
        
        self.publish_steering(steering_cmd)
        self.publish_wheel_speed(self.v)
        
        self.get_logger().info(
            f"📍 Pose: x={self.x:.3f}, y={self.y:.3f}, yaw={math.degrees(self.yaw):.2f}° | "
            f"🔄 Steering: {math.degrees(steering_cmd):.2f}° | 🚀 Speed: {self.v:.2f} m/s"
        )
        
        # For plotting
        self.robot_x.append(self.x)
        self.robot_y.append(self.y)
        self.update_plot()

    def quaternion_to_euler(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def publish_steering(self, steering):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['front_left_steering', 'front_right_steering']
        point = JointTrajectoryPoint()
        point.positions = [steering, steering]
        traj_msg.points.append(point)
        self.pub_steering.publish(traj_msg)

    def publish_wheel_speed(self, speed):
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [speed, speed]
        self.pub_wheel_spd.publish(wheel_msg)

    def update_plot(self):
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

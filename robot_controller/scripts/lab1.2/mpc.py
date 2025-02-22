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

# MPC parametersss
DT = 0.15      # time step [s]
N =   10        # horizon length (timesteps in horizon)
WB = 0.2       # [m] wheelbase of the robot

# 🎯 MANUAL MPC PARAMETERS (Using Optimized Values)
Q = np.diag([10.0, 10.0, 6.0, 2.0])   # State error penalty
Qf = Q                                 # Terminal cost
R = np.diag([3.0, 1.73436065])         # Control effort penalty
Rd = np.diag([0.2, 30.1])       # Smooth control transitions

class MPCPathFollower(Node):
    def __init__(self):
        super().__init__('mpc_path_follower')
        self.create_subscription(ModelStates, '/gazebo/model_states', self.gazebo_callback, 10)
        self.waypoints = self.load_path('/home/tang/ros2_lab1_m/src/FRA532_LAB1_6702_6703/robot_controller/data/path.yaml')
        self.get_logger().info("✅ Path loaded successfully.")
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
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
        if 'path' not in data:
            self.get_logger().error(f"⚠️ Key 'path' not found in {filename}")
            return []
        return np.array([(point['x'], point['y']) for point in data['path']])
    
    def should_skip_waypoint(self, current_pos, waypoint):
        """
        Returns True if the waypoint is too far or requires an impossible yaw change.
        """
        wp_x, wp_y = waypoint
        distance = np.linalg.norm(np.array([self.x, self.y]) - np.array([wp_x, wp_y]))
        
        yaw_to_wp = math.atan2(wp_y - self.y, wp_x - self.x)
        yaw_diff = self.normalize_angle(yaw_to_wp - self.yaw)

        # ✅ If the waypoint is too far or requires a large yaw change, skip it
        if distance > 3.0 or abs(yaw_diff) > np.deg2rad(75):
            return True
        return False
    
    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def get_reference_trajectory(self, current_pos, horizon, min_distance=0.2):
        ref_traj = []
        
       
        for i, wp in enumerate(self.waypoints):
            wp_x, wp_y = wp[0], wp[1]

            # ✅ Compute yaw if not provided
            if len(wp) > 2:
                wp_yaw = wp[2]
            else:
                wp_yaw = math.atan2(wp_y - current_pos[1], wp_x - current_pos[0])

            vector = np.array([wp_x, wp_y]) - np.array(current_pos)
            angle_to_wp = math.atan2(vector[1], vector[0])
            angle_diff = self.normalize_angle(angle_to_wp - self.yaw)

            max_angle_threshold = np.deg2rad(60)  # ✅ Allow ±60° in front
            if abs(angle_diff) > max_angle_threshold:
                continue

            # ✅ Skip very close waypoints
            distance_to_wp = np.linalg.norm(np.array([self.x, self.y]) - np.array([wp_x, wp_y]))
            if distance_to_wp < min_distance:
                continue  # ✅ Skip if too close

            # ✅ Skip waypoint if impossible to reach
            if self.should_skip_waypoint(current_pos, [wp_x, wp_y]):
                # self.get_logger().info(f"🚀 Skipping waypoint {i} at ({wp_x:.2f}, {wp_y:.2f})")
                continue  

            ref_traj.append([wp_x, wp_y, wp_yaw])
            if len(ref_traj) >= horizon:
                break

        # ✅ If not enough waypoints, repeat the last one
        if len(ref_traj) < horizon:
            last_wp = self.waypoints[-1]
            last_wp_yaw = last_wp[2] if len(last_wp) > 2 else 0.0
            ref_traj.extend([[last_wp[0], last_wp[1], last_wp_yaw]] * (horizon - len(ref_traj)))

        if ref_traj:
            next_wp_x, next_wp_y, next_wp_yaw = ref_traj[0]
            self.get_logger().info(f"🎯 Next Waypoint: x={next_wp_x:.3f}, y={next_wp_y:.3f}, θ={math.degrees(next_wp_yaw):.2f}°")

        return np.array(ref_traj)


    def mpc_control(self):
        """
        Compute control commands via MPC using iterative linearization.
        along with adjustment (adaptive) by adjusting the weight in the cost function (Q) according to the tracking error
        Model state: [x, y, yaw, v]
        Control inputs: [a, delta]
        """
        current_pos = [self.x, self.y]
        # 1) สร้าง reference trajectory (N+1 ขั้นตอนใน horizon)
        ref_traj = self.get_reference_trajectory(current_pos, horizon=N+1, min_distance=0.1)

        

        # 2) คำนวณ reference yaw จาก waypoints
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

        # 3) Reference speed
        ref_speed = self.target_speed

        # 4) estimate initial value velocity & steering
        v_nom = np.full(N, self.v)
        u_nom = np.zeros(N)  # for steering

        # ค่า nominal สำหรับ state trajectory (ใช้สำหรับ linearization)
        x_nom = np.full(N+1, self.x)
        y_nom = np.full(N+1, self.y)
        yaw_nom = np.full(N+1, self.yaw)

        # === Adaptive Part ===


        max_iter = 5
        tol = 1e-3

        sol = None  # กำหนด sol ให้มี scope นอก loop
        for it in range(max_iter):
            opti = ca.Opti()

            # กำหนดตัวแปร decision
            x_var   = opti.variable(N+1)
            y_var   = opti.variable(N+1)
            yaw_var = opti.variable(N+1)
            v_var   = opti.variable(N+1)
            a_var      = opti.variable(N)   # acceleration
            delta_var  = opti.variable(N)   # steering angle

            cost = 0

            # เงื่อนไขเริ่มต้น
            opti.subject_to(x_var[0] == self.x)
            opti.subject_to(y_var[0] == self.y)
            opti.subject_to(yaw_var[0] == self.yaw)
            opti.subject_to(v_var[0] == self.v)

            for k in range(N):
                # ค่า cost ของ state error (ใช้ Q_adapt ที่ปรับแล้ว)
                state_error = ca.vertcat(
                    x_var[k] - ref_traj[k, 0],
                    y_var[k] - ref_traj[k, 1],
                    yaw_var[k] - ref_yaw[k],
                    v_var[k] - ref_speed
                )
                cost += ca.mtimes([state_error.T, state_error])
                
                # ค่า cost ของ input effort
                u_vec = ca.vertcat(a_var[k], delta_var[k])
                cost += ca.mtimes([u_vec.T, R, u_vec])
                
                # ค่า cost สำหรับความต่อเนื่องของ input
                if k > 0:
                    input_diff = ca.vertcat(a_var[k] - a_var[k-1], delta_var[k] - delta_var[k-1])
                    cost += ca.mtimes([input_diff.T, Rd, input_diff])
                
                # Soft penalty สำหรับ steering angle ที่เกินขีดจำกัด
                big_lambda = 1215000.0
                penalty = big_lambda * (ca.fmax(0, ca.fabs(delta_var[k]) - self.max_steering_angle)**2)
                cost += penalty

                # Linearized dynamics: n use nominal ของ state ที่ timestep ปัจจุบัน
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

                # สำหรับ y
                linearized_y = y_var[k] + DT * (
                    v_nom_k * sin_yaw +
                    sin_yaw * (v_var[k] - v_nom_k) +
                    v_nom_k * cos_yaw * (yaw_var[k] - yaw_nom_k)
                )

                # สำหรับ yaw (ใช้ bilinear expansion)
                tan_nom = math.tan(delta_nom) if abs(delta_nom) < 1.4 else math.tan(1.4)
                bilinear_term = (
                    v_nom_k * tan_nom +
                    tan_nom * (v_var[k] - v_nom_k) +
                    v_nom_k * (1.0 / (math.cos(delta_nom) ** 2)) * (delta_var[k] - delta_nom)
                )
                linearized_yaw = yaw_var[k] + (DT / WB) * bilinear_term

                # สำหรับ velocity: v_{k+1} = v_k + DT * a_k
                linearized_v = v_var[k] + DT * a_var[k]

                # เพิ่ม constraint ของ dynamics สำหรับ state ถัดไป
                opti.subject_to(x_var[k+1] == linearized_x)
                opti.subject_to(y_var[k+1] == linearized_y)
                opti.subject_to(yaw_var[k+1] == linearized_yaw)
                opti.subject_to(v_var[k+1] == linearized_v)

                # Constraint สำหรับ input
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

            # ตั้งค่า solver (ใช้ IPOPT)
            p_opts = {"print_time": False}
            s_opts = {"print_level": 0}
            opti.solver("ipopt", p_opts, s_opts)
            try:
                sol = opti.solve()
            except RuntimeError as e:
                self.get_logger().error("❌ MPC solve error with CasADi: " + str(e))
                return 0.0, 0.0

            # ดึงค่าที่ได้จาก solution
            v_nom_new = np.array([sol.value(v_var[k]) for k in range(N)])
            delta_nom_new = np.array([sol.value(delta_var[k]) for k in range(N)])

            # จำลอง nominal state trajectory โดยใช้ input ปัจจุบัน
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

            # check nominal inputs
            if (np.linalg.norm(v_nom_new - v_nom) < tol) and (np.linalg.norm(delta_nom_new - u_nom) < tol):
                v_nom = v_nom_new
                u_nom = delta_nom_new
                break

            # update nominal values for next iteration 
            v_nom = v_nom_new
            u_nom = delta_nom_new

        # ใช้คำสั่งควบคุมตัวแรกจาก solution ที่ได้
        a_cmd = sol.value(a_var[0])
        delta_cmd = sol.value(delta_var[0])
        delta_cmd = np.clip(delta_cmd, -self.max_steering_angle, self.max_steering_angle)

        return a_cmd, delta_cmd


    def search_nearest_index(self):
        # คำนวณระยะ Euclidean จากตำแหน่งปัจจุบันกับทุก waypoint
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
        
        # show next waypoint 
        idx = self.search_nearest_index()
        dist = np.hypot(self.waypoints[idx, 0] - self.x,
                        self.waypoints[idx, 1] - self.y)
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
        
        # สำหรับ plotting
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

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
N = 7         # horizon length (จำนวน timesteps ใน horizon)
WB = 0.2      # [m] wheelbase of the robot

# Cost matrices (ปรับตามต้องการ)
Q = np.diag([1.0, 1.0, 0.1, 1.0])   # penalize error in [x, y, yaw, v]
Qf = Q                              # terminal state cost
R = np.diag([0.1, 0.2])             # penalize input effort [acceleration, steering]
Rd = np.diag([0.01, 0.01])          # penalize input difference

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
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.target_speed = 2.5  # [m/s]
        
        # Maximum steering angle (radians)
        self.max_steering_angle = 0.523598767 / 5  # ~15° (ปรับได้ตามความเหมาะสม)
        
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
        เลือก reference trajectory จาก waypoints โดยเริ่มจาก waypoint ที่ใกล้กับตำแหน่งปัจจุบัน
        และเลือก waypoint ที่อยู่ด้านหน้า robot (โดยใช้ heading)
        ถ้าไม่ครบ horizon ให้เติมด้วย waypoint สุดท้ายซ้ำ
        """
        heading = np.array([math.cos(self.yaw), math.sin(self.yaw)])
        ref_traj = []
        for wp in self.waypoints:
            vector = np.array(wp) - np.array(current_pos)
            # ตรวจสอบว่า waypoint อยู่ด้านหน้า robot หรือไม่
            if np.dot(vector, heading) < 0:
                continue
            # ถ้ามี waypoint แล้วตรวจสอบระยะห่าง
            if ref_traj:
                if np.linalg.norm(np.array(wp) - np.array(ref_traj[-1])) < min_distance:
                    continue
            ref_traj.append(wp)
            if len(ref_traj) >= horizon:
                break
        if len(ref_traj) < horizon:
            ref_traj.extend([self.waypoints[-1]] * (horizon - len(ref_traj)))
        return np.array(ref_traj)

    def mpc_control(self):
        """
        คำนวณ input ผ่าน MPC ด้วย iterative linearization
        Model state: [x, y, yaw, v]
        Control inputs: [a, delta]
        """
        current_pos = [self.x, self.y]
        # สร้าง reference trajectory (state references)
        ref_traj = self.get_reference_trajectory(current_pos, horizon=N+1, min_distance=0.1)
        
        # คำนวณ reference yaw จาก trajectory
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

        # กำหนดค่า nominal เริ่มต้นสำหรับ v และ u[1,k]
        v_nom = np.full(N, self.v)     # สมมุติใช้ความเร็วปัจจุบันสำหรับทุก timestep
        u_nom = np.zeros(N)            # สมมุติให้เริ่มแรก steering เป็น 0

        max_iter = 5
        tol = 1e-3  # tolerance สำหรับตรวจสอบ convergence

        for it in range(max_iter):
            # กำหนด optimization variables ใหม่ในแต่ละ iteration
            x_var = cp.Variable(N+1)
            y_var = cp.Variable(N+1)
            yaw_var = cp.Variable(N+1)
            v_var = cp.Variable(N+1)
            u = cp.Variable((2, N))  # u[0,:] = acceleration, u[1,:] = steering

            cost = 0
            constraints = []
            
            # เงื่อนไขเริ่มต้น
            constraints += [
                x_var[0] == self.x,
                y_var[0] == self.y,
                yaw_var[0] == self.yaw,
                v_var[0] == self.v
            ]
            
            for k in range(N):
                # คำนวณ state error สำหรับ cost
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
                
                # เพิ่ม soft penalty สำหรับ steering เกินขีดจำกัด
                lambda_ = 5000.0
                penalty = lambda_ * cp.square(cp.pos(cp.abs(u[1, k]) - self.max_steering_angle))
                cost += penalty
                
                # สำหรับ dynamics ของ x, y (ยังใช้ linearization จาก yaw ปัจจุบัน)
                cos_yaw0 = math.cos(self.yaw)
                sin_yaw0 = math.sin(self.yaw)
                constraints += [
                    x_var[k+1] == x_var[k] + DT * v_var[k] * cos_yaw0,
                    y_var[k+1] == y_var[k] + DT * v_var[k] * sin_yaw0
                ]
                
                # Linearize bilinear term v_var[k] * u[1, k] รอบ (v_nom[k], u_nom[k])
                # โดยใช้: v_nom*u_nom + u_nom*(v_var[k] - v_nom) + v_nom*(u[1,k] - u_nom)
                linearized_term = v_nom[k]*u_nom[k] + u_nom[k]*(v_var[k] - v_nom[k]) + v_nom[k]*(u[1, k] - u_nom[k])
                constraints += [
                    yaw_var[k+1] == yaw_var[k] + (DT / WB) * linearized_term,
                    v_var[k+1] == v_var[k] + DT * u[0, k]
                ]
                
                # ข้อจำกัดสำหรับ input
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
            
            # สร้างและแก้ปัญหา optimization
            prob = cp.Problem(cp.Minimize(cost), constraints)
            try:
                prob.solve(solver=cp.OSQP, warm_start=True)
            except Exception as e:
                self.get_logger().error(f"❌ MPC solve error: {e}")
                return 0.0, 0.0
            
            if prob.status not in ["optimal", "optimal_inaccurate"]:
                self.get_logger().warn("⚠️ MPC did not find an optimal solution")
                return 0.0, 0.0
            
            # ดึงค่า solution สำหรับ nominal update
            v_nom_new = np.array([v_var.value[k] for k in range(N)])
            u_nom_new = np.array([u.value[1, k] for k in range(N)])
            
            # ตรวจสอบ convergence
            if (np.linalg.norm(v_nom_new - v_nom) < tol) and (np.linalg.norm(u_nom_new - u_nom) < tol):
                v_nom = v_nom_new
                u_nom = u_nom_new
                break
            
            # อัปเดต nominal values สำหรับ iteration ถัดไป
            v_nom = v_nom_new
            u_nom = u_nom_new

        # ใช้คำสั่งใน timestep แรก
        a_cmd = u.value[0, 0]
        delta_cmd = u.value[1, 0]
        return a_cmd, delta_cmd


    def search_nearest_index(self):
        # คำนวณระยะห่างแบบ Euclidean ระหว่างตำแหน่งปัจจุบันของหุ่นและ waypoints ทั้งหมด
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
        self.ax.clear()
        self.ax.plot(self.waypoints[:, 0], self.waypoints[:, 1], 'go-', label="Planned Path")
        self.ax.plot(self.robot_x, self.robot_y, 'r.-', label="Actual Path")
        self.ax.scatter(self.robot_x[-1], self.robot_y[-1], c='purple', marker='x', label="Current Position")
        self.ax.set_title("MPC Controller")
        self.ax.legend()
        plt.draw()
        plt.pause(0.1)

def main():
    rclpy.init()
    node = MPCPathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

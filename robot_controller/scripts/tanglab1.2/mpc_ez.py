#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
import yaml
import numpy as np
import math
import cvxpy as cp  # ใช้สำหรับ Linear MPC

# Parameters for MPC
DT = 0.1      # time step [s]
N = 3        # horizon length
WB = 0.25      # [m] wheelbase of the robot

class LinearMPCROS(Node):

    def __init__(self):
        super().__init__('linear_mpc_ros')

        # Subscribe to Gazebo model states
        self.create_subscription(ModelStates, '/gazebo/model_states', self.gazebo_callback, 10)

        # Load waypoints from path.yaml
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
        self.target_speed = 5.5  # [m/s] Target speed

    def load_path(self, filename):
        """Load waypoints from YAML file."""
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
        if 'path' not in data:
            self.get_logger().error(f"⚠️ Key 'path' not found in {filename}")
            return []
        return np.array([(point['x'], point['y']) for point in data['path']])

    def search_nearest_index(self):
        """Find the index of the nearest waypoint."""
        distances = np.hypot(self.waypoints[:, 0] - self.x, self.waypoints[:, 1] - self.y)
        ind = np.argmin(distances)
        return ind

    def linear_mpc_control(self):
        """
        Compute control inputs using Linear MPC with linearized dynamics.
        Model state: [x, y, yaw, v]
        Control inputs: [a, delta] where:
            a     = acceleration
            delta = steering angle
        """
        # สร้าง reference trajectory จาก waypoints
        nearest_ind = self.search_nearest_index()
        ref_traj = []
        for i in range(N+1):
            idx = min(nearest_ind + i, len(self.waypoints)-1)
            ref_traj.append(self.waypoints[idx])
        ref_traj = np.array(ref_traj)
        
        # คำนวณ reference yaw จาก consecutive waypoints
        ref_yaw = []
        for i in range(len(ref_traj)-1):
            dy = ref_traj[i+1, 1] - ref_traj[i, 1]
            dx = ref_traj[i+1, 0] - ref_traj[i, 0]
            ref_yaw.append(math.atan2(dy, dx))
        ref_yaw.append(ref_yaw[-1])
        ref_yaw = np.array(ref_yaw)
        
        ref_speed = self.target_speed  # ความเร็วอ้างอิง

        # Linearize dynamics รอบๆ current yaw (ใช้ค่า current yaw ตลอด horizon)
        yaw0 = self.yaw
        cos_yaw0 = math.cos(yaw0)
        sin_yaw0 = math.sin(yaw0)

        # Define optimization variables
        x_var = cp.Variable(N+1)
        y_var = cp.Variable(N+1)
        yaw_var = cp.Variable(N+1)
        v_var = cp.Variable(N+1)
        a = cp.Variable(N)      # acceleration
        delta = cp.Variable(N)  # steering angle

        cost = 0
        constraints = []

        # Weights for cost function
        Q_pos = 1.0      # position error weight
        Q_yaw = 1.5      # yaw error weight
        Q_v = 1.0        # speed error weight
        R_u = 1.2        # control effort weight

        # Initial conditions
        constraints += [x_var[0] == self.x,
                        y_var[0] == self.y,
                        yaw_var[0] == self.yaw,
                        v_var[0] == self.v]

        for k in range(N):
            # เพิ่มค่า cost สำหรับ error และการใช้ control input
            cost += Q_pos * ((x_var[k] - ref_traj[k, 0])**2 + (y_var[k] - ref_traj[k, 1])**2)
            cost += Q_yaw * cp.square(yaw_var[k] - ref_yaw[k])
            cost += Q_v * cp.square(v_var[k] - ref_speed)
            cost += R_u * (cp.square(a[k]) + cp.square(delta[k]))

            # Linearized vehicle model dynamics:
            # x[k+1] = x[k] + DT * v[k] * cos(yaw0)
            # y[k+1] = y[k] + DT * v[k] * sin(yaw0)
            # yaw[k+1] = yaw[k] + (DT / WB) * (v[k] * delta[k])
            # v[k+1] = v[k] + DT * a[k]
            constraints += [
                x_var[k+1] == x_var[k] + DT * v_var[k] * cos_yaw0,
                y_var[k+1] == y_var[k] + DT * v_var[k] * sin_yaw0,
                yaw_var[k+1] == yaw_var[k] + (DT / WB) * (self.v * delta[k]),
                v_var[k+1] == v_var[k] + DT * a[k]
            ]
            # จำกัด input
            constraints += [
                cp.abs(delta[k]) <= 0.5236/3,  # approx 30 degrees in radians
                a[k] <= 1.0,
                a[k] >= -1.0
            ]

        # Terminal cost สำหรับสถานะสุดท้าย
        cost += Q_pos * ((x_var[N] - ref_traj[N, 0])**2 + (y_var[N] - ref_traj[N, 1])**2)
        cost += Q_yaw * cp.square(yaw_var[N] - ref_yaw[N])
        cost += Q_v * cp.square(v_var[N] - ref_speed)

        # Solve the optimization problem
        prob = cp.Problem(cp.Minimize(cost), constraints)
        try:
            prob.solve(solver=cp.OSQP, warm_start=True)
        except Exception as e:
            self.get_logger().error(f"❌ MPC solve error: {e}")
            return 0.0, 0.0  # คืนค่าควบคุมเป็น 0

        if prob.status not in ["optimal", "optimal_inaccurate"]:
            self.get_logger().warn("⚠️ MPC did not find an optimal solution")
            return 0.0, 0.0

        # เอาคำสั่งควบคุมตัวแรกออกมา
        a_cmd = a.value[0]
        delta_cmd = delta.value[0]
        return a_cmd, delta_cmd

    def quaternion_to_euler(self, q):
        """Convert quaternion to Euler yaw."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def gazebo_callback(self, msg):
        """Receive real-time pose data from Gazebo and apply Linear MPC control."""
        try:
            index = msg.name.index("limo")  # เปลี่ยนให้ตรงกับชื่อโมเดลของหุ่นยนต์คุณ
        except ValueError:
            self.get_logger().error("❌ Robot model not found in Gazebo!")
            return

        pose = msg.pose[index]
        self.x = pose.position.x
        self.y = pose.position.y
        self.yaw = self.quaternion_to_euler(pose.orientation)

        # ใช้ target speed ที่ตั้งไว้ (5.5 m/s)
        self.target_speed = 2.5  

        # คำนวณควบคุมผ่าน MPC
        acceleration, steering_cmd = self.linear_mpc_control()

        # Update speed ด้วย acceleration (Euler integration)
        self.v += acceleration * DT
        # จำกัดความเร็วไม่ให้เกิน target speed
        self.v = min(self.v, self.target_speed)

        # Publish control commands
        self.publish_steering(steering_cmd)
        self.publish_wheel_speed(self.v)

        self.get_logger().info(f"📍 Pose: x={self.x:.3f}, y={self.y:.3f}, yaw={math.degrees(self.yaw):.2f}° | "
                                 f"🔄 Steering: {math.degrees(steering_cmd):.2f}° | 🚀 Speed: {self.v:.2f} m/s")

    def publish_steering(self, steering):
        """Publish steering commands to Gazebo."""
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['front_left_steering', 'front_right_steering']
        point = JointTrajectoryPoint()
        point.positions = [steering, steering]
        traj_msg.points.append(point)
        self.pub_steering.publish(traj_msg)

    def publish_wheel_speed(self, speed):
        """Publish speed commands to Gazebo."""
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [speed, speed]
        self.pub_wheel_spd.publish(wheel_msg)

def main():
    rclpy.init()
    node = LinearMPCROS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

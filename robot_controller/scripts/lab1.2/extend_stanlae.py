#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
import numpy as np
import math
import yaml  # ใช้สำหรับโหลดไฟล์ YAML

def normalize_angle(angle):
    """Normalize angle to range [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi

class ExtendedStanleyController(Node):
    def __init__(self):
        super().__init__('extended_stanley_controller')
        self.dt_loop = 1 / 50.0  # Control loop at 20 Hz

        # Robot parameters
        self.wheel_base = 0.2           # ระยะห่างล้อหน้า-หลัง (m)
        self.track_width = 0.13         # ระยะห่างล้อซ้าย-ขวา (m)
        self.front_offset = 0.1  # ระยะจาก origin (กลางรถ) ไปยังล้อหน้า

        # Extended Stanley Control Gains
        self.k_stanley = 5.02          # Gain สำหรับ cross-track error
        self.k_soft = 2.2             # Softening term
        self.kd_yaw = 1.1             # Gain สำหรับ yaw rate damping
        self.kd_steer = 0.98           # Gain สำหรับ steering damping

        # Steering saturation limit (δ_max)
        self.delta_max = 0.5
        # Desired speed (m/s)
        self.desired_speed = 3.0

        # เก็บค่า steering จากรอบก่อนหน้า สำหรับ damping term
        self.prev_steering = 0.0

        # โหลด path จากไฟล์ YAML
        self.waypoints = self.load_path('/home/tang/ros2_lab1_m/src/FRA532_LAB1_6702_6703/robot_controller/data/path.yaml')  # ปรับ path ให้ถูกต้อง
        self.current_waypoint_index = 0  # waypoint ปัจจุบัน

        # Publishers
        self.pub_steering = self.create_publisher(
            JointTrajectory, '/joint_trajectory_position_controller/joint_trajectory', 10)
        self.pub_wheel_spd = self.create_publisher(
            Float64MultiArray, '/velocity_controllers/commands', 10)

        # Subscriber to Gazebo model states
        self.create_subscription(ModelStates, '/gazebo/model_states', self.gazebo_callback, 10)

    def load_path(self, file_path):
        """โหลด waypoints จากไฟล์ YAML"""
        try:
            with open(file_path, 'r') as file:
                data = yaml.safe_load(file)
            if 'path' in data:
                self.get_logger().info("Loaded {} waypoints".format(len(data['path'])))
                return data['path']
            else:
                self.get_logger().error("Key 'path' not found in file!")
                return []
        except Exception as e:
            self.get_logger().error("Failed to load path: {}".format(e))
            return []

    def gazebo_callback(self, msg):
        """ประมวลผล state ของหุ่นยนต์จาก Gazebo และคำนวณคำสั่งการเลี้ยว"""
        try:
            index = msg.name.index("limo")  # ตรวจสอบให้แน่ใจว่าใช้ชื่อโมเดลที่ถูกต้อง
        except ValueError:
            self.get_logger().error("Robot model not found in Gazebo!")
            return

        # รับค่า pose และ twist
        pose = msg.pose[index]
        twist = msg.twist[index]
        x, y = pose.position.x, pose.position.y
        _, _, yaw = self.quaternion_to_euler(pose.orientation)

        # คำนวณตำแหน่งล้อหน้า (front axle)
        front_x = x + self.front_offset * math.cos(yaw)
        front_y = y + self.front_offset * math.sin(yaw)

        # ความเร็วของหุ่นยนต์
        v = np.hypot(twist.linear.x, twist.linear.y)
        yaw_rate = twist.angular.z  # r_meas

        # รับ waypoint เป้าหมายจาก path
        target = self.get_target_waypoint(front_x, front_y, yaw)
        if target is None:
            self.get_logger().info("No more waypoints to follow")
            return

        target_x, target_y, target_yaw = target
        target_yaw_rate = 0.0  # สามารถคำนวณจาก waypoint ถัดไปได้ถ้ามี

        # คำนวณคำสั่ง steering โดยใช้ Extended Stanley Control
        steering = self.compute_extended_stanley(front_x, front_y, yaw, v, yaw_rate,
                                                   target_x, target_y, target_yaw, target_yaw_rate)

        # ส่งคำสั่งการเลี้ยวและความเร็ว
        self.publish_steering(steering)
        self.publish_wheel_speed(self.desired_speed)

    def get_target_waypoint(self, front_x, front_y, yaw):
        """
        เลือก waypoint เป้าหมายจาก self.waypoints โดยใช้ตำแหน่งของล้อหน้า
        และตรวจสอบให้แน่ใจว่า waypoint นั้นอยู่ด้านหน้าของรถ
        ถ้าระยะห่างระหว่าง front axle กับ waypoint ปัจจุบันน้อยกว่า threshold
        หรือ waypoint อยู่ด้านหลัง ให้เลื่อน index ไป waypoint ถัดไป
        """
        while self.current_waypoint_index < len(self.waypoints):
            wp = self.waypoints[self.current_waypoint_index]
            target_x = wp['x']
            target_y = wp['y']
            target_yaw = wp['yaw']
            
            # คำนวณเวกเตอร์จากตำแหน่งล้อหน้าไปยัง waypoint
            dx = target_x - front_x
            dy = target_y - front_y
            
            # คำนวณเวกเตอร์ heading ของรถ (ใช้ yaw)
            heading_vector = [math.cos(yaw), math.sin(yaw)]
            
            # คำนวณ dot product
            dot = dx * heading_vector[0] + dy * heading_vector[1]
            
            # ถ้า dot product น้อยกว่า 0 แสดงว่า waypoint อยู่ด้านหลัง ให้ข้าม waypoint นี้
            if dot < 0:
                self.get_logger().info("Waypoint {} is behind, skipping.".format(self.current_waypoint_index))
                self.current_waypoint_index += 1
                continue
            
            # คำนวณระยะห่างจากล้อหน้าไปยัง waypoint
            dist = math.hypot(dx, dy)
            threshold = 0.01  # 20 cm (ปรับตามความเหมาะสม)
            if dist < threshold:
                self.get_logger().info("Reached waypoint {} (distance: {:.3f} m)".format(self.current_waypoint_index, dist))
                self.current_waypoint_index += 1
                continue
            
            return target_x, target_y, target_yaw
        
        return None  # หมด waypoint แล้ว


    def compute_psi_ss(self, v, target_yaw_rate):
        """
        คำนวณ steady-state yaw (ψ_ss) สำหรับเส้นทางที่มีความโค้งคงที่
        (ในตัวอย่างนี้ใช้ค่า target_yaw_rate = 0 ทำให้ ψ_ss = 0)
        """
        k_ag = 1.0  # tuning constant (ปรับได้ตามต้องการ)
        return k_ag * v * target_yaw_rate

    def compute_extended_stanley(self, x, y, yaw, v, yaw_rate, target_x, target_y, target_yaw, target_yaw_rate):
        """คำนวณมุมเลี้ยวโดยใช้ Extended Stanley Control"""
        # คำนวณ cross-track error (e)
        dx = target_x - x
        dy = target_y - y
        cross_track_error = dx * (-math.sin(target_yaw)) + dy * math.cos(target_yaw)

        # คำนวณ steady-state yaw (ψ_ss)
        psi_ss = self.compute_psi_ss(v, target_yaw_rate)

        # เทอมแรก: (ψ - ψ_ss)
        heading_error = yaw - psi_ss  # เราใช้ heading_error เป็นชื่อที่ชัดเจนแทน heading_term

        # เพิ่ม log เพื่อตรวจสอบค่า cross-track error และ heading error
        self.get_logger().info("Cross-track error: {:.3f}, Heading error: {:.3f}".format(cross_track_error, heading_error))

        # เทอมที่สอง: arctan( k * e / (k_soft + v) )
        stanley_term = math.atan2(self.k_stanley * cross_track_error, self.k_soft + v)

        # เทอม damping สำหรับ yaw rate: k_d,yaw*(r_traj - r_meas)
        yaw_damping = self.kd_yaw * (target_yaw_rate - yaw_rate)

        # เทอม damping สำหรับ steering: k_d,steer*(δ_prev - (heading_error + stanley_term + yaw_damping))
        current_steering_est = heading_error + stanley_term + yaw_damping
        steer_damping = self.kd_steer * (self.prev_steering - current_steering_est)

        # รวมคำสั่ง steering ตามสมการ
        steering_cmd = heading_error + stanley_term + yaw_damping + steer_damping

        # ทำ saturation ไม่ให้เกิน ±δ_max
        steering_cmd = np.clip(steering_cmd, -self.delta_max, self.delta_max)

        # เก็บค่า steering command สำหรับรอบถัดไป
        self.prev_steering = steering_cmd

        return steering_cmd

    def publish_steering(self, steering):
        """เผยแพร่คำสั่ง steering ไปที่ Joint Trajectory Controller"""
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['front_left_steering', 'front_right_steering']
        point = JointTrajectoryPoint()
        point.positions = [steering, steering]
        traj_msg.points.append(point)
        self.pub_steering.publish(traj_msg)

    def publish_wheel_speed(self, speed):
        """เผยแพร่คำสั่งความเร็วให้กับ velocity controller"""
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [speed, speed]
        self.pub_wheel_spd.publish(wheel_msg)

    def quaternion_to_euler(self, q):
        """แปลง quaternion เป็น Euler angles (roll, pitch, yaw)"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return 0.0, 0.0, yaw

def main(args=None):
    rclpy.init(args=args)
    node = ExtendedStanleyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
import yaml
import numpy as np
from rclpy.duration import Duration

class PIDBicycleController(Node):
    def __init__(self):
        super().__init__('pid_bicycle_controller')
        self.dt_loop = 1/50.0  # Loop time in seconds

        # ✅ Robot parameters
        self.wheel_base = 0.2  
        self.wheel_radius = 0.045  
        self.max_steering_angle = 0.523598767/3  # 30 degrees in radians

        # ✅ PID Gains
        self.kp_steer = 1
        self.ki_steer = 0
        self.kd_steer = 0.1
        self.kp_speed = 1
        self.ki_speed = 0
        self.kd_speed = 0

        # ✅ PID Variables
        self.integral_steer = 0.0
        self.prev_error_steer = 0.0
        self.integral_speed = 0.0
        self.prev_error_speed = 0.0

        # ✅ Track visited waypoints 
        self.visited_points = set()

        # ✅ Load waypoints
        self.waypoints = self.load_path('/home/tang/ros2_lab1_m/src/FRA532_LAB1_6702_6703/robot_controller/data/path.yaml')

        # ✅ Publishers
        self.pub_steering = self.create_publisher(
            JointTrajectory, '/joint_trajectory_position_controller/joint_trajectory', 10)
        self.pub_wheel_spd = self.create_publisher(
            Float64MultiArray, '/velocity_controllers/commands', 10)
        
        # ✅ Subscriber to Gazebo model states
        self.create_subscription(ModelStates, '/gazebo/model_states', self.gazebo_callback, 10)

    def load_path(self, filename):
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
        if 'path' not in data:
            self.get_logger().error(f"Key 'path' not found in {filename}.")
            return []
        return [(point['x'], point['y'], point['yaw']) for point in data['path']]

    def nearest_waypoint(self, x, y, yaw):
        """
        เลือก waypoint ที่อยู่ด้านหน้าของหุ่นยนต์โดยใช้ dot product
        และ ignore waypoint ที่อยู่ใกล้เกิน threshold (เช่น < 0.05 m)
        """
        tolerance = 0.05          # if waypoint < 5 cm  ignore
        forward_threshold = 0.1   # waypoint should be front (dot product > 0.1)
        min_distance = float('inf')
        best_index = None

        for i, (wx, wy, _) in enumerate(self.waypoints):
            dx = wx - x
            dy = wy - y
            distance = np.hypot(dx, dy)
            if distance < tolerance:
                self.get_logger().info(f"⚠️ Ignoring waypoint {i} เพราะระยะ {distance:.3f} m น้อยกว่า {tolerance} m")
                continue
            robot_dir = np.array([np.cos(yaw), np.sin(yaw)])
            wp_vector = np.array([dx, dy])
            dot_product = np.dot(robot_dir, wp_vector)
            if dot_product > forward_threshold and distance < min_distance:
                min_distance = distance
                best_index = i

        if best_index is None:
            distances = [np.hypot(wx - x, wy - y) for wx, wy, _ in self.waypoints]
            best_index = int(np.argmin(distances))
            self.get_logger().warn(f"⚠️ ไม่พบ waypoint ที่ผ่านเงื่อนไข => เลือก waypoint {best_index} (fallback)")
        else:
            self.get_logger().info(f"🎯 เลือก waypoint {best_index} ที่มีระยะห่าง = {min_distance:.3f} m")
        return best_index

    def angle_difference(self, target_yaw, current_yaw):
        diff = target_yaw - current_yaw
        while diff > np.pi:
            diff -= 2 * np.pi
        while diff < -np.pi:
            diff += 2 * np.pi
        return diff

    def pid_control(self, error, prev_error, integral, kp, ki, kd, dt):
        derivative = (error - prev_error) / dt
        integral += error * dt
        return kp * error + ki * integral + kd * derivative, integral, error



    def gazebo_callback(self, msg):
        try:
            index = msg.name.index("limo")
        except ValueError:
            self.get_logger().error("Robot model not found in Gazebo!")
            return

        pose = msg.pose[index]
        x, y = pose.position.x, pose.position.y
        _, _, yaw = self.quaternion_to_euler(pose.orientation)

        # เลือก waypoint ที่เหมาะสม
        target_idx = self.nearest_waypoint(x, y, yaw)
        target_x, target_y, target_yaw = self.waypoints[target_idx]

        # คำนวณความผิดพลาดของตำแหน่ง
        error_speed = np.hypot(target_x - x, target_y - y)

        # กำหนด threshold ระยะห่าง (ปรับได้ตามความเหมาะสม)
        distance_threshold = 1.5  # ถ้า error_speed > 1.0 m ให้ใช้ Pure Pursuit ควบคุม





        # จำกัดสัญญาณ steering ให้อยู่ในช่วง ± max_steering_angle
        steering_cmd = np.clip(steering_cmd, -self.max_steering_angle, self.max_steering_angle)

        # คำนวณ PID สำหรับ speed (สามารถปรับเพิ่มส่วนนี้ได้ตามต้องการ)
        speed, self.integral_speed, self.prev_error_speed = self.pid_control(
            error_speed, self.prev_error_speed, self.integral_speed,
            self.kp_speed, self.ki_speed, self.kd_speed, self.dt_loop)
        speed = np.clip(speed, 5.0, 10.0)

        self.publish_steering(steering_cmd)
        self.publish_wheel_speed(speed)

        self.get_logger().info(f"Target WP: x={target_x:.3f}, y={target_y:.3f}, yaw={target_yaw:.3f}")
        self.get_logger().info(f"Steering: {steering_cmd:.3f} rad, Speed: {speed:.3f} m/s")
        self.get_logger().info(f"✅ Waypoint {target_idx}/{len(self.waypoints)} reached!")

    def publish_steering(self, steering):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['front_left_steering', 'front_right_steering']
        point = JointTrajectoryPoint()
        point.positions = [steering, steering]
        point.time_from_start = Duration(seconds=0.1).to_msg()
        traj_msg.points.append(point)
        self.pub_steering.publish(traj_msg)

    def publish_wheel_speed(self, speed):
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [speed, speed]
        self.pub_wheel_spd.publish(wheel_msg)

    def quaternion_to_euler(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return 0, 0, yaw

def main(args=None):
    rclpy.init(args=args)
    node = PIDBicycleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

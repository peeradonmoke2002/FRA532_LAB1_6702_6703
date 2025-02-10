#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class AckermannController(Node):
    def __init__(self):
        super().__init__('ackermann_controller')
        
        # Robot parameters
        self.wheel_base = 0.2         # Distance between front and rear axles (meters)
        self.track_width = 0.14       # Distance between left and right wheels (meters)
        self.max_steering_angle = 0.523598767  # 30 degrees in radians
        self.wheel_radius = 0.045      # Rear wheel radius (meters)

        # Initialize variables
        self.cmd_vel = [0.0, 0.0]
        self.wheel_speed = 0.0
        self.steer_angles = 0.0
        self.ackermann_percentage = 0.0
        
        # Create publisher for steering commands
        self.pub_steering = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_position_controller/joint_trajectory',
            10
        )
        # Publisher for drive (wheel speed) commands
        self.pub_wheel_spd = self.create_publisher(
            Float64MultiArray, 
            '/velocity_controllers/commands', 
            10
        )

        self.sub_joint_state = self.create_subscription(
            JointState,
            '/joint_states',  
            self.cmd_steer_callback,
            10
        )
        
        # Subscribe to cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_timer(0.05, self.timer_callback)

    def cmd_vel_callback(self, msg):
        self.cmd_vel = [msg.linear.x, msg.angular.z]

    def cmd_steer_callback(self, msg):
        steer = msg.position
        # self.steer_angles = np.array(steer)
        # self.steer_angles = np.array([self.steer_angles[3], self.steer_angles[5]])

    
    def publish_steering(self, left_angle, right_angle):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['front_left_steering', 'front_right_steering']
        
        point = JointTrajectoryPoint()
        point.positions = [float(left_angle), float(right_angle)]
        point.time_from_start = rclpy.duration.Duration(seconds=0.1).to_msg()
        
        traj_msg.points.append(point)
        self.pub_steering.publish(traj_msg)
        # self.get_logger().info(
        #     f"Steering command: left = {left_angle:.3f} rad, right = {right_angle:.3f} rad"
        # )

    def compute_ackermann_angles(self, steering_angle):
        """Calculate left and right steering angles using Ackermann geometry."""
        if steering_angle == 0:
            return 0.0, 0.0
        
        tan_steer = np.tan(steering_angle)
        
        # Standard formulas:
        left_angle = np.arctan(
            (self.wheel_base * tan_steer) / 
            (self.wheel_base - 0.5 * self.track_width * tan_steer)
        )
        right_angle = np.arctan(
            (self.wheel_base * tan_steer) / 
            (self.wheel_base + 0.5 * self.track_width * tan_steer)
        )
        
        # Apply steering angle limits
        left_angle = np.clip(left_angle, -self.max_steering_angle, self.max_steering_angle)
        right_angle = np.clip(right_angle, -self.max_steering_angle, self.max_steering_angle)
        # self.get_logger().info(
        #     f"Computed Ackermann Angles: center_steer = {steering_angle:.3f} rad, "
        #     f"left_angle = {left_angle:.3f} rad, right_angle = {right_angle:.3f} rad"
        # )
        return left_angle, right_angle

    def timer_callback(self):
        # Update wheel speed (assuming wheel_speed = linear velocity / wheel radius)
        self.wheel_speed = self.cmd_vel[0] / self.wheel_radius

        # If the robot is moving straight (or almost straight),
        # set steering angles to zero.
        if abs(self.cmd_vel[0]) < 1e-9 or abs(self.cmd_vel[1]) < 1e-99:
            self.steer_angles = 0.0
            left_angle, right_angle = 0.0, 0.0
        else:
            # Compute the instantaneous turning radius (ICR)
            r_ICR = self.cmd_vel[0] / self.cmd_vel[1]
            # Compute the center steering angle using the bicycle model:
            # δ = arctan(L / R)
            self.steer_angles = np.arctan(self.wheel_base / r_ICR)
            # Compute the left and right steering angles using Ackermann geometry.
            left_angle, right_angle = self.compute_ackermann_angles(self.steer_angles)
            # cal ackermann percentage
            ak = np.arctan(self.wheel_base/ (self.wheel_base/np.tan(right_angle) - self.track_width))
            self.ackermann_percentage = (left_angle / ak) * 100
            print(self.ackermann_percentage)

        
        # Publish the steering commands.
        self.publish_steering(left_angle, right_angle)
        

        # Publish the wheel speeds (assuming both wheels receive the same command).
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [self.wheel_speed, self.wheel_speed]
        self.pub_wheel_spd.publish(wheel_msg)



def main(args=None):
    rclpy.init(args=args)
    node = AckermannController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
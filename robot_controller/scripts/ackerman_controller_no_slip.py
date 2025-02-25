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
        
        self.wheel_base = 0.2         # L: Distance between front and rear axles (meters)
        self.wheel_radius = 0.045     # r: Rear wheel radius (meters)
        self.track_width = 0.14       # Distance between left and right wheels (meters)
        self.max_steering_angle = np.deg2rad(30)  # Maximum steering angle (30 degrees in radians)
        # Initialize variables
        self.cmd_vel = [0.0, 0.0]
        self.wheel_speed = 0.0
        self.steer_angles = 0.0
        self.ackermann_percentage = 0.0
        
        # Create publisher for steering commands
        self.pub_steering = self.create_publisher(
            Float64MultiArray,
            "/position_controllers/commands",
            10
        )
        # Publisher for drive (wheel speed) commands
        self.pub_wheel_spd = self.create_publisher(
            Float64MultiArray, 
            '/velocity_controllers/commands', 
            10
        )
        
        # Subscribe to cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_timer(0.01, self.timer_callback)

    def cmd_vel_callback(self, msg):
        self.cmd_vel = [msg.linear.x, msg.angular.z]

    
    def publish_steering(self, steering_L: float, steering_R: float):
        steering_msg = Float64MultiArray()
        steering_msg.data = [steering_L, steering_R]
        self.pub_steering.publish(steering_msg)

    def publish_wheel_speed(self, wheelspeed_L:float, wheelspeed_R:float):
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [wheelspeed_L, wheelspeed_R]
        self.pub_wheel_spd.publish(wheel_msg)

    
    def compute_ackermann_angles(self, center_steer: float):
        if abs(center_steer) < 1e-6:
            return 0.0, 0.0

        sin_steer = np.sin(center_steer)
        cos_steer = np.cos(center_steer)
        tan_steer = np.tan(center_steer)
        # inside = np.arctan(
        #     (2 * self.wheel_base * sin_steer) /
        #     (2 * self.wheel_base * cos_steer - self.track_width * sin_steer)
        # )
        # outside = np.arctan(
        #     (2 * self.wheel_base * sin_steer) /
        #     (2 * self.wheel_base * cos_steer + self.track_width * sin_steer)
        # )
        inside = np.arctan(
            (self.wheel_base * np.tan(center_steer)) /
            (self.wheel_base - 0.5 * self.track_width * np.tan(center_steer))
        )
        outside = np.arctan(
            (self.wheel_base * np.tan(center_steer)) /
            (self.wheel_base + 0.5 * self.track_width * np.tan(center_steer))
        )

        return inside, outside



    def timer_callback(self):
        # Compute wheel speed (assume wheel speed = forward speed / wheel radius)
        v_x = self.cmd_vel[0]  # forward linear speed
        # omega speed each wheel
        wheel_speed_left = v_x / self.wheel_radius
        wheel_speed_right = v_x / self.wheel_radius

        # Check for straight or nearly straight motion:
        if abs(self.cmd_vel[0]) < 1e-9 or abs(self.cmd_vel[1]) < 1e-9:
            self.steer_angle_center = 0.0
            left_angle, right_angle = 0.0, 0.0
            self.publish_steering(left_angle, right_angle)
 
        else:
            # base from tan(max_steering_angle) = L/R
            r_ICR = self.wheel_base / np.tan(self.max_steering_angle) 
            turn_sign = np.sign(self.cmd_vel[1])  # +1 for left turn, -1 for right turn
            self.steer_angle_center = np.arctan(self.wheel_base / r_ICR) * turn_sign
            
            if turn_sign > 0:
                # turn left
                inside_angle, outside_angle = self.compute_ackermann_angles(self.steer_angle_center)
                left_angle =  inside_angle
                right_angle = outside_angle
                # print(left_angle,right_angle)
            else:
                # turn right
                outside_angle, inside_angle = self.compute_ackermann_angles(self.steer_angle_center)
                left_angle = outside_angle
                right_angle = inside_angle
                # print(left_angle,right_angle)
            self.publish_steering(left_angle, right_angle)


        self.publish_wheel_speed(wheel_speed_left, wheel_speed_right)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




            # if turn_sign > 0:
            #     wheel_speed_left = self.cmd_vel[0] * (2 - (self.track_width / r_ICR)) / (2 * self.wheel_radius)
            #     wheel_speed_right = self.cmd_vel[0] * (2 + (self.track_width / r_ICR)) / (2 * self.wheel_radius)
            # else:
            #     wheel_speed_left = self.cmd_vel[0] * (2 + (self.track_width / r_ICR)) / (2 * self.wheel_radius)
            #     wheel_speed_right = self.cmd_vel[0] * (2 - (self.track_width / r_ICR)) / (2 * self.wheel_radius)













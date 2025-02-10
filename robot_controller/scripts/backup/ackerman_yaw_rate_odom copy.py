#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (Point, Pose, PoseWithCovariance, Quaternion, Twist, 
                               TwistWithCovariance, Vector3, TransformStamped)
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Float64MultiArray
from tf2_ros import TransformBroadcaster
import tf_transformations
from sensor_msgs.msg import JointState
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException, TransformBroadcaster
import math
from rclpy.constants import S_TO_NS
import numpy as np

class YawrateOdom(Node):
    def __init__(self):
        super().__init__('YawrateOdom')
        queue_size = 10
        self.dt_loop = 1/50.0
        self.prev_time = self.get_clock().now()
        self.create_timer(0.05, self.timer_callback)

        # Robot parameters
        self.wheel_base = 0.2         # Distance between front and rear axles (meters)
        self.track_width = 0.14       # Distance between left and right wheels (meters)
        self.max_steering_angle = 0.523598767  # 30 degrees in radians
        self.wheel_radius = 0.045      # Rear wheel radius (meters)
        self.wheel_omega = np.array([0.0, 0.0])

        # Initialize variables
        self.robot_twist = [0.0, 0.0]
        self.robot_position = [0.0, 0.0, 0.0]  # x, y, theta
        self.sent_linear_velocity = 0.0
        self.sent_angular_velocity = 0.0
        self.steering_angles = np.array([0.0, 0.0])

        # Spawn values (as strings, then converted to floats)
        spawn_x_val = "9.073500"
        spawn_y_val = "0.0"
        spawn_yaw_val = "1.57"
        self.robot_position = [float(spawn_x_val), float(spawn_y_val), float(spawn_yaw_val)]
        z_spawn = math.sin(float(spawn_yaw_val)/2.0)
        w_spawn = math.cos(float(spawn_yaw_val)/2.0) 

        self.pose_cov = np.diag([1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9])
        self.twist_cov = np.diag([1.0e-9, 1.0e-6, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9])

        # Publisher
        self.wheel_pub = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_br = TransformBroadcaster(self)
        self.isOdomUpdate = False

        # Initialize the odometry output message using spawn values
        self.odom_output = Odometry(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id='odom'
            ),
            child_frame_id='base_footprint',  # Use base_footprint consistently
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(
                        x=float(spawn_x_val),
                        y=float(spawn_y_val),
                        z=0.0
                    ),
                    orientation=Quaternion(
                        # Correct conversion: use spawn_yaw_val for yaw
                        x=0.0,
                        y=0.0,
                        z= z_spawn,
                        w= w_spawn
                    )
                ),
                covariance=self.pose_cov.flatten().tolist()
            ),
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(
                        x=0.0,
                        y=0.0,
                        z=0.0
                    ),
                    angular=Vector3(
                        z=0.0
                    )
                ),
                covariance=self.twist_cov.flatten().tolist()
            )
        )

    def joint_state_callback(self, msg):
        self.wheel_omega = msg.velocity
        # Extract left and right wheel velocities (indices 2 and 4)
        self.wheel_omega = np.array([self.wheel_omega[2], self.wheel_omega[4]])

    def cmd_steer_callback(self, msg):
        self.steering_angles = msg.position
        self.steering_angles = np.array([self.steering_angles[3], self.steering_angles[5]])

    def calculate_wheel_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / S_TO_NS
        self.prev_time = current_time

        # Apply forward kinematics to get twist (linear velocity & yaw rate)
        self.robot_twist = self.forward_kinematics_ackermann(
            self.wheel_omega[0], self.steering_angles[0], self.steering_angles[1]
        )

        v = self.robot_twist[0]
        omega = self.robot_twist[1]

        if abs(omega) > 1e-6:
            # Use circular motion equations when turning
            dx = v / omega * (math.sin(self.robot_position[2] + omega * dt) - math.sin(self.robot_position[2]))
            dy = v / omega * (-math.cos(self.robot_position[2] + omega * dt) + math.cos(self.robot_position[2]))
        else:
            # Straight-line motion
            dx = v * math.cos(self.robot_position[2]) * dt
            dy = v * math.sin(self.robot_position[2]) * dt

        dtheta = omega * dt

        self.robot_position[0] += dx
        self.robot_position[1] += dy
        self.robot_position[2] += dtheta

        self.robot_position[2] = math.atan2(math.sin(self.robot_position[2]), math.cos(self.robot_position[2]))  # Normalize yaw


    def pub_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.robot_position[0]
        t.transform.translation.y = self.robot_position[1]
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.robot_position[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_br.sendTransform(t)

    def publish_odometry(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"
        msg.pose.pose.position.x = self.robot_position[0]
        msg.pose.pose.position.y = self.robot_position[1]
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.robot_position[2])
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.twist.twist.linear.x = self.robot_twist[0]
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = self.robot_twist[1]
        msg.pose.covariance = self.pose_cov.flatten().tolist()
        msg.twist.covariance = self.twist_cov.flatten().tolist()
        self.odom_pub.publish(msg)

    def forward_kinematics_ackermann(self, rear_wheel_velocity, left_steering_angle, right_steering_angle):
        v = rear_wheel_velocity * self.wheel_radius  # Compute linear velocity

        if abs(left_steering_angle) < 1e-6 and abs(right_steering_angle) < 1e-6:
            return np.array([v, 0.0])  # Moving straight
        
        # Compute turning radius for each wheel
        R_left = self.wheel_base / np.tan(left_steering_angle)
        R_right = self.wheel_base / np.tan(right_steering_angle)

        # Compute effective turning radius of the **rear axle midpoint**
        R_eff = (R_left + R_right) / 2.0

        # Compute angular velocity (yaw rate)
        omega = v / R_eff  

        return np.array([v, omega])


    def timer_callback(self):
        if self.isOdomUpdate:
            self.isOdomUpdate = False
        else:
            self.calculate_wheel_odometry()
            self.pub_transform()
            self.publish_odometry()
            self.robot_twist = self.forward_kinematics_ackermann(
                self.wheel_omega[0],
                self.steering_angles[0],
                self.steering_angles[1]
            )
            # print(self.robot_twist)

def main(args=None):
    rclpy.init(args=args)
    pub_odom_node = YawrateOdom()
    rclpy.spin(pub_odom_node)
    pub_odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

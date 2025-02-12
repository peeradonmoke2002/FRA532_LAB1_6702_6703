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
import numpy as np

class YawrateOdom(Node):
    def __init__(self):
        super().__init__('YawrateOdom')
        self.dt_loop = 1/50.0
        self.prev_time = self.get_clock().now()
        self.create_timer(self.dt_loop, self.timer_callback)

        # Robot parameters
        self.wheel_base = 0.2         # Distance between front and rear axles (meters)
        self.track_width = 0.14       # Distance between left and right wheels (meters)
        self.max_steering_angle = 0.523598767  # 30 degrees in radians
        self.wheel_radius = 0.045     # Rear wheel radius (meters)

        # State variables
        # robot_position: [x, y, theta] (theta in radians)
        self.robot_position = np.array([0.0, 0.0, 0.0])
        # orientation stored as a quaternion (computed from theta)
        self.orientation = tf_transformations.quaternion_from_euler(0.0, 0.0, self.robot_position[2])
        # Steering angles for the left and right wheels (radians)
        self.steering_angles = np.array([0.0, 0.0])
        # Rear wheel angular velocities (rad/s)
        self.wheel_omega = np.array([0.0, 0.0])

        # Spawn values (as strings, then converted to floats)
        spawn_x_val = "9.073500"
        spawn_y_val = "0.0"
        spawn_yaw_val = "1.57"
        self.robot_position = np.array([float(spawn_x_val), float(spawn_y_val), float(spawn_yaw_val)])
        self.orientation = tf_transformations.quaternion_from_euler(0.0, 0.0, self.robot_position[2])
        self.pose_cov = np.diag([1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9])
        self.twist_cov = np.diag([1.0e-9, 1.0e-6, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9])

        # Publishers and subscribers
        self.wheel_pub = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        # TF2 broadcaster
        self.tf_br = TransformBroadcaster(self)

 

    def joint_state_callback(self, msg):
        # Extract rear wheel velocities (assumed indices 2 and 4)
        self.wheel_omega = np.array([msg.velocity[2], msg.velocity[4]])
        # Extract steering angles (assumed indices 3 and 5)
        self.steering_angles = np.array([msg.position[3], msg.position[5]])

    # ---------------------------------------------------------------------------
    # Forward Kinematics Functions
    # ---------------------------------------------------------------------------
    def forward_kinematics_calc(self, pos, v_g, steering, dt):
        if np.abs(steering) < 1e-6:
            # Straight-line motion
            new_x = pos[0] + v_g * np.cos(pos[2]) * dt
            new_y = pos[1] + v_g * np.sin(pos[2]) * dt
            new_theta = pos[2]
        else:
            # For turning motion, the effective turning radius is:
            R = self.wheel_base / np.tan(steering)
            omega = v_g / R
            dtheta = omega * dt
            theta = pos[2] + dtheta
            new_x = R * np.sin((v_g/R)*dt + pos[2]) + pos[0] - R*np.sin(pos[2])
            new_y = -R * np.cos((v_g/R)*dt + pos[2]) + R *np.cos(pos[2]) + pos[1]
            new_theta = theta
        return np.array([new_x, new_y, new_theta])

    def forward_kinematics(self, dt):
        """
        Compute the forward kinematics update:
          - Compute linear speed from the average rear wheel angular velocity.
          - Compute the average steering angle.
          - Update the robot position using the closed-form solution.
          - Update the orientation quaternion from the new heading.
        """
        # Compute average rear wheel angular velocity and linear speed (v = omega * radius)
        v_outer = self.wheel_omega[0] * self.wheel_radius
        v_inner = self.wheel_omega[1] * self.wheel_radius 
        v_g = (v_outer + v_inner) / 2.0
        
        # Compute average steering angle from left and right steering values.
        steering_avg = np.mean(self.steering_angles)
        
        # Compute the new state (x, y, theta) using forward kinematics.
        new_state = self.forward_kinematics_calc(self.robot_position, v_g, steering_avg, dt)
        self.robot_position = new_state
        # Update the orientation quaternion from the new heading.
        self.orientation = tf_transformations.quaternion_from_euler(0.0, 0.0, float(new_state[2]))
    # ---------------------------------------------------------------------------
    
    def pub_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = float(self.robot_position[0])
        t.transform.translation.y = float(self.robot_position[1])
        t.transform.translation.z = 0.0
        t.transform.rotation.x = self.orientation[0]
        t.transform.rotation.y = self.orientation[1]
        t.transform.rotation.z = self.orientation[2]
        t.transform.rotation.w = self.orientation[3]
        self.tf_br.sendTransform(t)

    def publish_odometry(self) -> Odometry:
        # Compute linear speed and angular speed for the twist.
        v_outer = self.wheel_omega[0] * self.wheel_radius
        v_inner = self.wheel_omega[1] * self.wheel_radius 
        v_g = (v_outer + v_inner) / 2.0
        theta = self.robot_position[2]
        vx = v_g * np.cos(theta)
        vy = v_g * np.sin(theta)
        steering_avg = np.mean(self.steering_angles)
        if np.abs(steering_avg) < 1e-6:
            angular_speed = 0.0
        else:
            R = self.wheel_base / np.tan(steering_avg)
            angular_speed = v_g / R
            
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        odom_msg.pose.pose.position = Point(
            x=float(self.robot_position[0]),
            y=float(self.robot_position[1]),
            z=0.0
        )
        odom_msg.pose.pose.orientation = Quaternion(
            x=self.orientation[0],
            y=self.orientation[1],
            z=self.orientation[2],
            w=self.orientation[3]
        )
        odom_msg.twist.twist.linear = Vector3(x=vx, y=vy, z=0.0)
        odom_msg.twist.twist.angular =  Vector3(x=0.0, y=0.0, z=angular_speed)
        odom_msg.pose.covariance = self.pose_cov.flatten()
        odom_msg.twist.covariance = self.twist_cov.flatten()
        return odom_msg

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        self.prev_time = current_time
        self.forward_kinematics(dt)
        self.pub_transform()
        odom_msg = self.publish_odometry()
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YawrateOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

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
import math
from ackermann_interfaces.msg import AckermannFeedback  # not used in this integration
from rclpy.constants import S_TO_NS
from scipy.spatial.transform import Rotation
import numpy as np
from typing import NamedTuple

# A simple state structure
class AckermannState(NamedTuple):
    position: np.array           # [x, y, theta] (theta in radians)
    orientation: Rotation        # as a scipy Rotation (about z)
    left_wheel_speed: float      # (rad/s)
    right_wheel_speed: float     # (rad/s)
    steering_angle: float        # (radians) – using one front steering angle
    time: rclpy.time.Time

class YawrateOdom(Node):
    def __init__(self):
        super().__init__('YawrateOdom')
        self.dt_loop = 1/50.0
        self.prev_time = self.get_clock().now()
        self.create_timer(0.05, self.timer_callback)

        # Robot parameters
        self.wheel_base = 0.2         # Distance between front and rear axles (meters)
        self.track_width = 0.14       # Distance between left and right wheels (meters)
        self.max_steering_angle = 0.523598767  # 30 degrees in radians
        self.wheel_radius = 0.045      # Rear wheel radius (meters)
        self.wheel_omega = np.array([0.0, 0.0])  # rear wheel angular velocities

        # Initialize variables (these are also stored in self.state)
        self.robot_twist = [0.0, 0.0]         # [linear_velocity, angular_velocity]
        self.robot_position = [0.0, 0.0, 0.0]   # [x, y, theta]
        self.steering_angles = np.array([0.0, 0.0])  # [left_steering_angle, right_steering_angle]

        # Spawn values (as strings, then converted to floats)
        spawn_x_val = "9.073500"
        spawn_y_val = "0.0"
        spawn_yaw_val = "1.57"
        self.robot_position = [float(spawn_x_val), float(spawn_y_val), float(spawn_yaw_val)]
        z_spawn = math.sin(float(spawn_yaw_val)/2.0)
        w_spawn = math.cos(float(spawn_yaw_val)/2.0) 

        self.pose_cov = np.diag([1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9])
        self.twist_cov = np.diag([1.0e-9, 1.0e-6, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9])

        # Publishers
        self.wheel_pub = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_br = TransformBroadcaster(self)
        self.isOdomUpdate = False

        # Initialize odometry message (for publishing later)
        self.odom_output = Odometry(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id='odom'
            ),
            child_frame_id='base_footprint',
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(
                        x=float(spawn_x_val),
                        y=float(spawn_y_val),
                        z=0.0
                    ),
                    orientation=Quaternion(
                        x=0.0,
                        y=0.0,
                        z=z_spawn,
                        w=w_spawn
                    )
                ),
                # covariance=self.pose_cov.flatten()
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
                # covariance=self.twist_cov.flatten()
            )
        )

        # Initialize our state using the spawn values.
        self.state = AckermannState(
            position=np.array(self.robot_position),
            orientation=Rotation.from_euler('z', self.robot_position[2]),
            left_wheel_speed=0.0,
            right_wheel_speed=0.0,
            steering_angle=0.0,
            time=self.get_clock().now()
        )

    def joint_state_callback(self, msg):
        # Extract rear wheel velocities (assumed at indices 2 and 4)
        self.wheel_omega = np.array([msg.velocity[2], msg.velocity[4]])
        # Extract steering angles (assumed at indices 3 and 5)
        self.steering_angles = np.array([msg.position[3], msg.position[5]])

    def turn_radius(self, steering_angle: float) -> float:
        """
        For a bicycle model: R = wheel_base / tan(delta).  
        Returns infinity if steering angle is (nearly) zero.
        """
        if abs(steering_angle) < 1e-6:
            return math.inf
        else:
            return self.wheel_base / math.tan(steering_angle)

    def linear_velocity(self, orientation: Rotation, speed: float) -> np.array:
        """Rotate the forward speed into the global frame."""
        return orientation.apply([speed, 0, 0])  # assumes vehicle forward is +x

    def update_odometry_closed_form(self):
        """
        Update the vehicle state using closed-form integration under the constant-curvature assumption.
        If turning:  
            R = v/omega, then
            theta_new = theta_old + omega*dt  
            X_new = X_old + R*(sin(theta_new) - sin(theta_old))
            Y_new = Y_old - R*(cos(theta_new) - cos(theta_old))
        If moving straight:  
            X_new = X_old + v*cos(theta_old)*dt  
            Y_new = Y_old + v*sin(theta_old)*dt  
            theta_new = theta_old
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds * 1e-9

        # Compute linear velocity from average rear wheel angular velocity
        avg_rear_omega = (self.wheel_omega[0] + self.wheel_omega[1]) / 2.0
        v = avg_rear_omega * self.wheel_radius

        # avg_steer
        avg_steer = (self.steering_angles[0] + self.steering_angles[1]) / 2.0

        # Determine if steering is effectively zero.
        if abs(self.steering_angles[0]) < 1e-6 and abs(self.steering_angles[1]) < 1e-6:
            # Straight-line motion
            omega = 0.0
            theta_old = self.state.orientation.as_euler('zyx')[0]
            theta_new = theta_old
            X_old, Y_old, _ = self.state.position
            X_new = X_old + v * math.cos(theta_old) * dt
            Y_new = Y_old + v * math.sin(theta_old) * dt
        else:
            # Compute effective turning radius using the two steering angles.
            R_left = self.wheel_base / math.tan(self.steering_angles[0])
            R_right = self.wheel_base / math.tan(self.steering_angles[1])
            R_eff = (R_left + R_right) / 2.0
            # Angular velocity (omega) from the bicycle model:
            omega = v / R_eff
            theta_old = self.state.orientation.as_euler('zyx')[0]
            theta_new = theta_old + omega * dt
            X_old, Y_old, _ = self.state.position
            X_new = X_old + R_eff * (math.sin(theta_new) - math.sin(theta_old))
            Y_new = Y_old - R_eff * (math.cos(theta_new) - math.cos(theta_old))

        new_position = np.array([X_new, Y_new, theta_new])
        new_orientation = Rotation.from_euler('z', theta_new)
        self.state = AckermannState(
            position=new_position,
            orientation=new_orientation,
            left_wheel_speed=self.wheel_omega[0],
            right_wheel_speed=self.wheel_omega[1],
            steering_angle=avg_steer,
            time=current_time
        )
        self.prev_time = current_time

    def pub_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = float(self.state.position[0])
        t.transform.translation.y = float(self.state.position[1])
        t.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, float(self.state.position[2]))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_br.sendTransform(t)

    def publish_odometry(self) -> Odometry:
        """Build an Odometry message from self.state."""
        state = self.state
        quaternion = state.orientation.as_quat()
        # Compute linear velocity from average rear wheel speeds.
        linear_speed = self.wheel_radius * (state.left_wheel_speed + state.right_wheel_speed) / 2.0
        linear_velocity = self.linear_velocity(state.orientation, linear_speed)
        # Compute angular speed using the bicycle model (if steering is nonzero).
        if abs(state.steering_angle) < 1e-6:
            angular_speed = 0.0
        else:
            R_eff = self.turn_radius(state.steering_angle)
            angular_speed = linear_speed / R_eff

        odom_msg = Odometry()
        odom_msg.header.stamp = state.time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        odom_msg.pose.pose.position = Point(
            x=float(state.position[0]),
            y=float(state.position[1]),
            z=0.0
        )
        odom_msg.pose.pose.orientation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )
        odom_msg.twist.twist.linear = Vector3(
            x=linear_velocity[0],
            y=linear_velocity[1],
            z=linear_velocity[2]
        )
        odom_msg.twist.twist.angular = Vector3(
            x=0.0,
            y=0.0,
            z=angular_speed
        )
        #odom_msg.pose.covariance = self.pose_cov.flatten()
        #odom_msg.twist.covariance = self.twist_cov.flatten()
        return odom_msg

    def timer_callback(self):
        # Update our state using closed-form (exact) integration for constant curvature.
        self.update_odometry_closed_form()
        self.pub_transform()
        odom_msg = self.publish_odometry()
        self.odom_pub.publish(odom_msg)
        self.isOdomUpdate = True

def main(args=None):
    rclpy.init(args=args)
    pub_odom_node = YawrateOdom()
    rclpy.spin(pub_odom_node)
    pub_odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

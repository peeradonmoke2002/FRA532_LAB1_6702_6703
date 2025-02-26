#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
import math

# -----------------------------
# Define Noise Covariances
# -----------------------------
# Process noise covariance Q (15x15) decrease value = high precision
Q = np.diag([
    0.02, 0.02, 0.02,            # position noise
    np.deg2rad(0.1), np.deg2rad(0.1), np.deg2rad(0.1),  # orientation noise (rad) roll pitch yaw
    0.1, 0.1, 0.1,               # linear velocity noise
    np.deg2rad(0.1), np.deg2rad(0.1), np.deg2rad(0.1),  # angular velocity noise (rad/s)
    0.2, 0.2, 0.2                # linear acceleration noise
]) ** 2

# Measurement noisecovariance for odometry (6x6): [p (3), v (3)]
R_odom = np.diag([0.3, 0.3, 0.3, 0.3, 0.3, 0.3]) ** 2

# Measurement noise covariance for IMU (9x9): [orientation (3), angular velocity (3), linear acceleration (3)]
R_imu = np.diag([
    np.deg2rad(0.1), np.deg2rad(0.1), np.deg2rad(0.1),
    np.deg2rad(0.1), np.deg2rad(0.1), np.deg2rad(0.1),
    0.2, 0.2, 0.2
]) ** 2

print('Noise covariances defined.')

# -----------------------------
# Helper Functions
# -----------------------------
def R_from_euler(roll, pitch, yaw):
    """Compute the rotation matrix from Euler angles."""
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]
    ])

def J_from_euler(roll, pitch, yaw):
    """Compute Jacobian mapping angular velocities to Euler angle rates (for ZYX angles)."""
    cos_pitch = math.cos(pitch)
    if abs(cos_pitch) < 1e-4:
        cos_pitch = 1e-4
    return np.array([
        [1, math.sin(roll)*math.tan(pitch), math.cos(roll)*math.tan(pitch)],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll)/cos_pitch, math.cos(roll)/cos_pitch]
    ])

def dR_droll(roll, pitch, yaw):
    cr = math.cos(roll); sr = math.sin(roll)
    cp = math.cos(pitch); sp = math.sin(pitch)
    cy = math.cos(yaw); sy = math.sin(yaw)
    Rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [0,   0,  1]])
    Ry = np.array([[cp, 0, sp],
                   [0,  1, 0],
                   [-sp,0, cp]])
    dRx = np.array([[0, 0, 0],
                    [0, -sr, -cr],
                    [0, cr, -sr]])
    return Rz @ Ry @ dRx

def dR_dpitch(roll, pitch, yaw):
    cr = math.cos(roll); sr = math.sin(roll)
    cp = math.cos(pitch); sp = math.sin(pitch)
    cy = math.cos(yaw); sy = math.sin(yaw)
    Rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [0,   0,  1]])
    dRy = np.array([[-sp, 0, cp],
                    [0, 0, 0],
                    [-cp,0, -sp]])
    Rx = np.array([[1,0,0],
                   [0,cr,-sr],
                   [0,sr,cr]])
    return Rz @ dRy @ Rx

def dR_dyaw(roll, pitch, yaw):
    cr = math.cos(roll); sr = math.sin(roll)
    cp = math.cos(pitch); sp = math.sin(pitch)
    cy = math.cos(yaw); sy = math.sin(yaw)
    dRz = np.array([[-sy, -cy, 0],
                    [cy, -sy, 0],
                    [0, 0, 0]])
    Ry = np.array([[cp,0,sp],
                   [0,1,0],
                   [-sp,0,cp]])
    Rx = np.array([[1,0,0],
                   [0,cr,-sr],
                   [0,sr,cr]])
    return dRz @ Ry @ Rx

def dJ_droll(roll, pitch, yaw):
    cos_roll = math.cos(roll); sin_roll = math.sin(roll)
    tan_pitch = math.tan(pitch)
    dJ = np.zeros((3,3))
    dJ[0,1] = cos_roll*tan_pitch
    dJ[0,2] = -sin_roll*tan_pitch
    dJ[1,1] = -sin_roll
    dJ[1,2] = -cos_roll
    dJ[2,1] = cos_roll/math.cos(pitch)
    dJ[2,2] = -sin_roll/math.cos(pitch)
    return dJ

def dJ_dpitch(roll, pitch, yaw):
    sin_roll = math.sin(roll); cos_roll = math.cos(roll)
    cos_pitch = math.cos(pitch)
    sec_pitch2 = 1.0/(cos_pitch**2)
    dJ = np.zeros((3,3))
    dJ[0,1] = sin_roll*sec_pitch2
    dJ[0,2] = cos_roll*sec_pitch2
    dJ[2,1] = sin_roll*math.sin(pitch)/(cos_pitch**2)
    dJ[2,2] = cos_roll*math.sin(pitch)/(cos_pitch**2)
    return dJ

def dJ_dyaw(roll, pitch, yaw):
    return np.zeros((3,3))

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

# -----------------------------
# Dynamic Model and Jacobian
# -----------------------------
def dynamic_model(x, dt, u_alpha):
    """
    Compute new state from current state using a kinematic model.
    The state vector x is 15x1:
      [p (3), r (3), v (3), ω (3), a (3)]
    u_alpha is the control input for angular acceleration (3x1).
    """
    x_new = np.zeros((15,1))
    roll = x[3,0]; pitch = x[4,0]; yaw = x[5,0]
    R_mat = R_from_euler(roll, pitch, yaw)
    b = x[6:9] * dt + 0.5 * x[12:15] * (dt**2)
    x_new[0:3] = x[0:3] + R_mat @ b
    J_mat = J_from_euler(roll, pitch, yaw)
    x_new[3:6] = x[3:6] + J_mat @ x[9:12] * dt
    x_new[6:9] = x[6:9] + x[12:15] * dt
    x_new[9:12] = x[9:12] + u_alpha * dt
    x_new[12:15] = x[12:15]
    return x_new

def jacobian_F(x, dt):
    F = np.eye(15)
    I3 = np.eye(3)
    roll = x[3,0]; pitch = x[4,0]; yaw = x[5,0]
    R_mat = R_from_euler(roll, pitch, yaw)
    dR_dr = np.zeros((3,3,3))
    dR_dr[:,:,0] = dR_droll(roll, pitch, yaw)
    dR_dr[:,:,1] = dR_dpitch(roll, pitch, yaw)
    dR_dr[:,:,2] = dR_dyaw(roll, pitch, yaw)
    b = x[6:9] * dt + 0.5 * x[12:15] * (dt**2)
    L = np.zeros((3,3))
    for i in range(3):
        L[:, i] = (dR_dr[:,:,i] @ b).flatten()
    F[0:3, 3:6] = L
    F[0:3, 6:9] = R_mat * dt
    F[0:3, 12:15] = R_mat * (0.5 * dt**2)
    J_mat = J_from_euler(roll, pitch, yaw)
    dJ = dJ_droll(roll, pitch, yaw)*x[9,0] + dJ_dpitch(roll, pitch, yaw)*x[10,0] + dJ_dyaw(roll, pitch, yaw)*x[11,0]
    F[3:6, 3:6] = np.eye(3) + dJ * dt
    F[3:6, 9:12] = J_mat * dt
    F[6:9, 6:9] = I3
    F[6:9, 12:15] = I3 * dt
    F[9:12, 9:12] = I3
    F[12:15, 12:15] = I3
    return F

def ekf_predict(xEst, PEst, dt, Q, u_alpha):
    F = jacobian_F(xEst, dt)
    xPred = dynamic_model(xEst, dt, u_alpha)
    PPred = F @ PEst @ F.T + Q
    return xPred, PPred

def ekf_update_odom(xEst, PEst, z, R_odom):
    # z: measurement vector [p_x, p_y, p_z, v_x, v_y, v_z]^T
    H = np.zeros((6, 15))
    H[0:3, 0:3] = np.eye(3)  # position
    H[3:6, 6:9] = np.eye(3)  # velocity
    zPred = H @ xEst
    y = z - zPred
    S = H @ PEst @ H.T + R_odom
    K = PEst @ H.T @ np.linalg.inv(S)
    xEst_new = xEst + K @ y
    PEst_new = (np.eye(15) - K @ H) @ PEst
    return xEst_new, PEst_new

def ekf_update_imu(xEst, PEst, z, R_imu):
    # z: measurement vector [roll, pitch, yaw, ω_x, ω_y, ω_z, a_x, a_y, a_z]^T
    H = np.zeros((9, 15))
    H[0:3, 3:6] = np.eye(3)      # orientation
    H[3:6, 9:12] = np.eye(3)     # angular velocity
    H[6:9, 12:15] = np.eye(3)    # linear acceleration
    zPred = H @ xEst
    y = z - zPred
    for i in range(3):
        y[i, 0] = normalize_angle(y[i, 0])
    S = H @ PEst @ H.T + R_imu
    K = PEst @ H.T @ np.linalg.inv(S)
    xEst_new = xEst + K @ y
    PEst_new = (np.eye(15) - K @ H) @ PEst
    for i in range(3):
        xEst_new[3+i,0] = normalize_angle(xEst_new[3+i,0])
    return xEst_new, PEst_new

print('EKF functions defined.')

# -----------------------------
# EKF Node: Odom Filtered
# -----------------------------
class OdomFilteredNode(Node):
    def __init__(self):
        super().__init__('odom_filtered_node')
        # Initialize state (15x1 vector): [p(3), r(3), v(3), ω(3), a(3)]
        self.xEst = np.zeros((15,1))
        self.PEst = np.eye(15) * 1e-3
        # Control input (for angular acceleration update); assume zero control here.
        self.u_alpha = np.zeros((3,1))
        self.last_time = self.get_clock().now()
        self.dt = 0.02  # 50 Hz prediction rate

        # Subscribers for raw odometry and IMU measurements
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Publisher for filtered odometry
        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)

        # Timer for the prediction step
        self.create_timer(self.dt, self.timer_callback)

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # EKF Prediction
        self.xEst, self.PEst = ekf_predict(self.xEst, self.PEst, dt, Q, self.u_alpha)
        # Publish filtered odometry
        self.publish_filtered_odom()

    def odom_callback(self, msg):
        # Extract odometry measurement: position and linear velocity.
        z = np.zeros((6,1))
        z[0,0] = msg.pose.pose.position.x
        z[1,0] = msg.pose.pose.position.y
        z[2,0] = msg.pose.pose.position.z
        z[3,0] = msg.twist.twist.linear.x
        z[4,0] = msg.twist.twist.linear.y
        z[5,0] = msg.twist.twist.linear.z
        # EKF update with odometry measurement
        self.xEst, self.PEst = ekf_update_odom(self.xEst, self.PEst, z, R_odom)

    def imu_callback(self, msg):
        # Extract IMU measurement: orientation, angular velocity, and linear acceleration.
        z = np.zeros((9,1))
        # Orientation: convert quaternion to Euler angles (roll, pitch, yaw)
        q = msg.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        z[0,0] = roll
        z[1,0] = pitch
        z[2,0] = yaw
        # Angular velocity:
        z[3,0] = msg.angular_velocity.x
        z[4,0] = msg.angular_velocity.y
        z[5,0] = msg.angular_velocity.z
        # Linear acceleration:
        z[6,0] = msg.linear_acceleration.x
        z[7,0] = msg.linear_acceleration.y
        z[8,0] = msg.linear_acceleration.z
        # EKF update with IMU measurement
        self.xEst, self.PEst = ekf_update_imu(self.xEst, self.PEst, z, R_imu)

    def publish_filtered_odom(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        # Position from state (first three entries)
        odom_msg.pose.pose.position.x = self.xEst[0,0]
        odom_msg.pose.pose.position.y = self.xEst[1,0]
        odom_msg.pose.pose.position.z = self.xEst[2,0]
        # Orientation from state (roll, pitch, yaw in entries 3-5)
        quat = quaternion_from_euler(self.xEst[3,0], self.xEst[4,0], self.xEst[5,0])
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        # Optionally, fill the pose covariance (6x6 flattened)
        cov = [0.0]*36
        cov[0] = self.PEst[0,0]
        cov[7] = self.PEst[1,1]
        cov[14] = self.PEst[2,2]
        cov[21] = self.PEst[3,3]
        cov[28] = self.PEst[4,4]
        cov[35] = self.PEst[5,5]
        odom_msg.pose.covariance = cov
        # Use the estimated linear velocity from state (entries 6-8)
        odom_msg.twist.twist.linear.x = self.xEst[6,0]
        odom_msg.twist.twist.linear.y = self.xEst[7,0]
        odom_msg.twist.twist.linear.z = self.xEst[8,0]
        # For simplicity, twist covariance is set to zero.
        odom_msg.twist.covariance = [0.0]*36
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomFilteredNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

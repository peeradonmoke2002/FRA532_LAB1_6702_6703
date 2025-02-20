import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Pose, Point, Vector3
import numpy as np
import tf_transformations
import tf2_ros

class KinematicOdometryNode(Node):
    def __init__(self):
        super().__init__("kinematic_odometry")

        # 🚗 Vehicle Parameters
        self.WHEEL_BASE = 0.2  # Distance between front and rear axles (m)
        self.TRACK_WIDTH = 0.13  # Distance between left and right wheels (m)
        self.WHEEL_RADIUS = 0.04815  # Wheel radius (m)
        self.DT = 0.05  # Time step (s)

        # 🔄 Odometry Scaling
        self.ODO_SCALE = 0.9  # Scale for rear wheel speed correction
        self.YAW_SCALE = 0.335  # Scale for yaw rate correction (reduces fast turning)

        # 🌍 Initial Pose and Motion Variables
        self.x = 9.073500
        self.y = 0.0
        self.theta = 1.57
        self.beta = 0.0  # Side-slip angle
        self.v = 0.0
        self.omega = 0.0  # Raw yaw rate (rad/s)
        self.filtered_omega = 0.0  # Filtered yaw rate (rad/s)

        # 🔄 Filter Parameters
        self.alpha = 0.7  # More smoothing on yaw rate

        # 🚀 ROS 2 Subscriptions and Publishers
        self.sub = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # ⏳ Timer for Odometry Publishing
        self.timer = self.create_timer(self.DT, self.publish_odometry)

        # ✅ TF Broadcaster for odom → base_footprint
        self.tf_br = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("✅ Kinematic Odometry Node Started!")

    def joint_state_callback(self, msg: JointState):
        try:
            # 🔗 Extract Joint Indices
            idx_rl = msg.name.index("rear_left_wheel")
            idx_rr = msg.name.index("rear_right_wheel")
            idx_fl = msg.name.index("front_left_wheel")
            idx_fr = msg.name.index("front_right_wheel")
            idx_fl_steer = msg.name.index("front_left_steering")
            idx_fr_steer = msg.name.index("front_right_steering")

            # 🏎️ Extract Wheel Velocities (Convert rad/s → m/s)

            r_fl = np.array([ self.TRACK_WIDTH / 2, self.WHEEL_BASE / 2])
            r_fr = np.array([-self.TRACK_WIDTH / 2, self.WHEEL_BASE / 2])
            r_rl = np.array([ self.TRACK_WIDTH / 2, -self.WHEEL_BASE / 2])
            r_rr = np.array([-self.TRACK_WIDTH / 2, -self.WHEEL_BASE / 2])

            v_rl = msg.velocity[idx_rl] * self.WHEEL_RADIUS
            v_rr = msg.velocity[idx_rr] * self.WHEEL_RADIUS
            v_fl = msg.velocity[idx_fl] * self.WHEEL_RADIUS
            v_fr = msg.velocity[idx_fr] * self.WHEEL_RADIUS

            # 🔄 Extract Steering Angles (Radians)
            delta_fl = msg.position[idx_fl_steer]
            delta_fr = msg.position[idx_fr_steer]

            # ✅ Compute Vehicle Linear Velocity (Use ALL Wheels)
            v_k = (r_fl[0] * v_fl * np.sin(delta_fl) + r_fr[0] * v_fr * np.sin(delta_fr) + r_rl[0] * v_rl + r_rr[0] * v_rr) / 4.0


            # ✅ Compute Yaw Rate (ω_k) using ALL Wheels
            omega_k = (
                (v_fl * np.sin(delta_fl) + v_fr * np.sin(delta_fr) - v_rl * np.sin(0) - v_rr * np.sin(0))
                / self.TRACK_WIDTH
            ) * self.YAW_SCALE

            # ✅ Apply Exponential Moving Average (EMA) Filter to Yaw Rate
            self.filtered_omega = self.alpha * omega_k + (1 - self.alpha) * self.filtered_omega

            # ✅ Compute Side-Slip Angle (β)
            if abs(self.v) > 0.01:
                self.beta = np.arctan((self.WHEEL_BASE / 2) * self.filtered_omega / self.v)
            else:
                self.beta = 0.0

            self.get_logger().info(f"✅ Filtered Yaw Rate (ω): {self.filtered_omega:.4f} rad/s, Linear Vel (v_k): {self.v:.4f} m/s")

        except ValueError as e:
            self.get_logger().warn(f"⚠️ Joint name not found: {e}")

    def publish_odometry(self):
        # 📍 Update Pose Using Filtered Yaw Rate (ω_k)
        self.x += self.v * self.DT * np.cos(self.beta + self.theta + (self.filtered_omega * self.DT / 2))
        self.y += self.v * self.DT * np.sin(self.beta + self.theta + (self.filtered_omega * self.DT / 2))
        self.theta += self.filtered_omega * self.DT

        # ✅ Convert yaw to quaternion
        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta)

        # ✅ Publish Odometry Message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose = Pose(
            position=Point(x=self.x, y=self.y, z=0.0),
            orientation=Quaternion(
                x=quaternion[0],
                y=quaternion[1],
                z=quaternion[2],
                w=quaternion[3]
            )
        )
        odom_msg.twist.twist.linear = Vector3(x=self.v, y=0.0, z=0.0)
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=self.filtered_omega)

        self.odom_pub.publish(odom_msg)
        self.tf_br.sendTransform(TransformStamped())

def main(args=None):
    rclpy.init(args=args)
    node = KinematicOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

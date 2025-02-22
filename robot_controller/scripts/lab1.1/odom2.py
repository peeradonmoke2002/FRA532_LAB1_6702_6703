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
        self.WHEEL_BASE = 0.2  # Distance between front and rear wheels (m)
        self.TRACK_WIDTH = 0.13  # Distance between left and right wheels (m)
        self.WHEEL_RADIUS = 0.04815  # Wheel radius (m)
        self.DT = 0.1  # Time step (s)

        # 🌍 Initial Pose and Motion Variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.beta = 0.0  # Side-slip angle
        self.v = 0.0
        self.omega = 0.0

        # 🚀 ROS 2 Subscriptions and Publishers
        self.sub = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # ⏳ Timer for Odometry Publishing
        self.timer = self.create_timer(self.DT, self.publish_odometry)

        # ✅ TF Broadcaster for odom → base_footprint
        self.tf_br = tf2_ros.TransformBroadcaster(self)

    def joint_state_callback(self, msg: JointState):
        try:
            # 🔗 Extract Joint Indices
            idx_fl = msg.name.index("front_left_wheel")
            idx_fr = msg.name.index("front_right_wheel")
            idx_rl = msg.name.index("rear_left_wheel")
            idx_rr = msg.name.index("rear_right_wheel")
            idx_fl_steer = msg.name.index("front_left_steering")
            idx_fr_steer = msg.name.index("front_right_steering")

            # 🏎️ Extract Wheel Velocities (Convert rad/s → m/s)
            v_fl = msg.velocity[idx_fl] * self.WHEEL_RADIUS
            v_fr = msg.velocity[idx_fr] * self.WHEEL_RADIUS
            v_rl = msg.velocity[idx_rl] * self.WHEEL_RADIUS
            v_rr = msg.velocity[idx_rr] * self.WHEEL_RADIUS

            # 🔄 Extract Steering Angles (Radians)
            delta_fl = msg.position[idx_fl_steer]
            delta_fr = msg.position[idx_fr_steer]

            # 📌 Compute Vehicle Linear Velocity (Average Rear Wheels)
            self.v = (v_rl + v_rr) / 2.0

            # 📌 Compute Angular Velocity (Yaw Rate)
            self.omega = ((v_fr * np.tan(delta_fr) + v_fl * np.tan(delta_fl)) / self.WHEEL_BASE)

            # 📌 Compute Side-Slip Angle (Equation with Empirical Correction)
            if abs(self.v) > 0.01:
                self.beta = np.arctan((self.WHEEL_BASE / 2) * self.omega / self.v)
            else:
                self.beta = 0.0

        except ValueError as e:
            self.get_logger().warn(f"⚠️ Joint name not found: {e}")

    def publish_odometry(self):
        # 📍 Update Pose with Corrected Side-Slip
        self.x += self.v * self.DT * np.cos(self.beta + self.theta + self.omega * self.DT / 2)
        self.y += self.v * self.DT * np.sin(self.beta + self.theta + self.omega * self.DT / 2)
        self.theta += self.omega * self.DT

        # ✅ Convert yaw to quaternion
        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta)

        # ✅ Publish Odometry message
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
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=self.omega)

        self.odom_pub.publish(odom_msg)

        # ✅ Publish transform (odom → base_footprint)
        transform = TransformStamped()
        transform.header.stamp = odom_msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_br.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = KinematicOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

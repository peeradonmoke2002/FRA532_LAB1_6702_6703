import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Pose, Point
import tf_transformations
import numpy as np

from tf2_ros import TransformBroadcaster

class SingleTrackModelNode(Node):
    def __init__(self):
        super().__init__('single_track_model_node')

        # ✅ Subscribe to /joint_states for wheel velocities & steering angles
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.get_logger().info("✅ Subscribed to /joint_states")

        # ✅ Publish odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # ✅ TF broadcaster for odom → base_footprint
        self.tf_br = TransformBroadcaster(self)

        # 🚗 Vehicle parameters
        self.x, self.y, self.theta = 9.073500, 0.0,  1.57   # Position and heading
        self.wheelbase = 0.2  # Distance between front and rear axles (m)
        self.wheel_radius = 0.04815  # Wheel radius (m)

        # Wheel velocities (rear wheels)
        self.wheel_omega = np.array([0.0, 0.0])  # [RL, RR]

        # Steering angles (front wheels)
        self.steering_angles = np.array([0.0, 0.0])  # [FL, FR]

        # 🔄 Timer for state update (10 Hz)
        self.timer = self.create_timer(0.05, self.update_state)

    def joint_state_callback(self, msg):
        """Extract rear wheel velocities and front steering angles from /joint_states"""
        # ✅ Use only REAR wheels to calculate speed
        self.wheel_omega = np.array([msg.velocity[4], msg.velocity[2]]) * self.wheel_radius

        # ✅ Use only FRONT wheels for steering
        self.steering_angles = np.array([msg.position[3], msg.position[5]])

    def compute_yaw_rate(self, v, delta_F):
        """Compute yaw rate ω using kinematic single-track model"""
        return (v / self.wheelbase) * np.tan(delta_F)

    def update_state(self):
        """Update the vehicle state using the single-track model"""
        delta_t = 0.035  # Time step (s)
  
        # ✅ Compute vehicle speed from rear wheels
        v_k = np.mean(self.wheel_omega)

        # ✅ Compute steering angle from front wheels
        delta_F = np.mean(self.steering_angles)

        # ✅ Compute yaw rate using kinematics
        omega_k = self.compute_yaw_rate(v_k, delta_F) 

        # ✅ Update position using kinematic equations
        x_next = self.x + v_k * delta_t * np.cos(self.theta + (omega_k * delta_t / 2))
        y_next = self.y + v_k * delta_t * np.sin(self.theta + (omega_k * delta_t / 2))
        theta_next = self.theta + omega_k * delta_t

        # ✅ Update state variables
        self.x, self.y, self.theta = x_next, y_next, theta_next

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
        odom_msg.twist.twist.linear = Vector3(x=v_k, y=0.0, z=0.0)
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=omega_k)

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

        self.get_logger().info(f"📡 Odometry → X: {self.x:.3f}, Y: {self.y:.3f}, Theta: {np.degrees(self.theta):.3f}°, v: {v_k:.3f} m/s, ω: {omega_k:.3f} rad/s")

def main(args=None):
    rclpy.init(args=args)
    node = SingleTrackModelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

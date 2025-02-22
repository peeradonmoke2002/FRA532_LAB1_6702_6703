import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np

class SlipDetectionController(Node):
    def __init__(self):
        super().__init__('slip_detection_controller')

        # Parameters
        self.wheel_radius = 0.045  # Wheel radius (meters)
        self.track_width = 0.14  # Distance between left and right wheels (meters)

        # Slip detection parameters
        self.delta_w_min = 0.15  # Min difference in wheel speeds to detect slip
        self.sigma_w_max = 0.20  # Max std deviation for stability
        self.smoothing_window = 5  # Number of samples for smoothing
        self.velocity_history = {"left": [], "right": []}  # Store velocity history

        # Variables
        self.wheel_speeds = {"left": 0.0, "right": 0.0}  # Measured wheel speeds
        self.cmd_vel = {"linear": 0.0, "angular": 0.0}  # Commanded velocity
        self.correction_factor = 0.9  # Reduce speed of slipping wheel

        # ROS 2 Subscribers
        self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        # ROS 2 Publisher
        self.pub_wheel_speed = self.create_publisher(Float64MultiArray, "/velocity_controllers/commands", 10)

        # Timer
        self.create_timer(0.1, self.timer_callback)  # Run every 100ms

    def joint_states_callback(self, msg):
        """ Read actual wheel speeds from /joint_states """
        try:
            # Extract wheel speeds from joint states
            wheel_names = msg.name
            velocities = msg.velocity

            # Find indices for left and right wheels
            left_index = wheel_names.index("rear_left_wheel")
            right_index = wheel_names.index("rear_right_wheel")

            # Store wheel speeds
            self.wheel_speeds["left"] = velocities[left_index] * self.wheel_radius
            self.wheel_speeds["right"] = velocities[right_index] * self.wheel_radius

            # Update history for smoothing
            self.velocity_history["left"].append(self.wheel_speeds["left"])
            self.velocity_history["right"].append(self.wheel_speeds["right"])

            # Keep history size limited
            if len(self.velocity_history["left"]) > self.smoothing_window:
                self.velocity_history["left"].pop(0)
                self.velocity_history["right"].pop(0)

        except ValueError:
            self.get_logger().error("Wheel names mismatch in /joint_states")

    def cmd_vel_callback(self, msg):
        """ Read desired velocity from /cmd_vel """
        self.cmd_vel["linear"] = msg.linear.x
        self.cmd_vel["angular"] = msg.angular.z

    def detect_slip(self):
        """ Detects if either wheel is slipping based on speed differences """
        if len(self.velocity_history["left"]) < self.smoothing_window:
            return {"left": False, "right": False}  # Not enough data

        # Compute smoothed wheel speeds
        left_avg = np.mean(self.velocity_history["left"])
        right_avg = np.mean(self.velocity_history["right"])
        delta_w = abs(left_avg - right_avg)  # Difference in wheel speeds
        sigma_w = np.std([left_avg, right_avg])  # Standard deviation of wheel speeds

        slip_left = delta_w > self.delta_w_min and sigma_w < self.sigma_w_max
        slip_right = delta_w > self.delta_w_min and sigma_w < self.sigma_w_max

        return {"left": slip_left, "right": slip_right}

    def adjust_wheel_speeds(self):
        """ Adjusts the speeds to correct slip """
        slip = self.detect_slip()
        adjusted_speeds = {
            "left": self.cmd_vel["linear"] - (self.cmd_vel["angular"] * self.track_width / 2),
            "right": self.cmd_vel["linear"] + (self.cmd_vel["angular"] * self.track_width / 2)
        }

        if slip["left"]:
            self.get_logger().info("Left wheel slipping! Reducing speed.")
            adjusted_speeds["left"] *= self.correction_factor

        if slip["right"]:
            self.get_logger().info("Right wheel slipping! Reducing speed.")
            adjusted_speeds["right"] *= self.correction_factor

        return adjusted_speeds

    def publish_wheel_speeds(self):
        """ Publishes the corrected wheel speeds to the controller """
        speeds = self.adjust_wheel_speeds()
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [speeds["left"] / self.wheel_radius, speeds["right"] / self.wheel_radius]
        self.pub_wheel_speed.publish(wheel_msg)

    def timer_callback(self):
        """ Runs at a fixed interval to check and correct wheel speeds """
        self.publish_wheel_speeds()

def main(args=None):
    rclpy.init(args=args)
    node = SlipDetectionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

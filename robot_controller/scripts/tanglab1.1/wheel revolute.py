import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class WheelRevolutionTracker(Node):
    def __init__(self):
        super().__init__('wheel_revolution_tracker')

        # Robot parameters
        self.wheel_revolutions = {  # Store revolutions for each wheel
            "front_left_wheel": 0.0,
            "front_right_wheel": 0.0,
            "rear_right_wheel": 0.0,
            "rear_left_wheel": 0.0
        }
        self.last_time = None  # To store previous timestamp

        # ROS 2 Subscriber
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

    def joint_state_callback(self, msg: JointState):
        """ รับค่าจาก /joint_states และคำนวณจำนวนรอบของล้อแต่ละล้อ """
        if self.last_time is None:
            self.last_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            return  # Skip first iteration

        # คำนวณเวลาที่ผ่านไป (delta_t)
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        delta_t = current_time - self.last_time
        self.last_time = current_time

        if delta_t <= 0:
            return  # Skip if time is invalid

        # Mapping ชื่อ Joint กับค่าความเร็ว
        wheel_names = ["front_left_wheel", "front_right_wheel", "rear_right_wheel", "rear_left_wheel"]
        wheel_velocities = {}

        for i, name in enumerate(msg.name):
            if name in wheel_names:
                wheel_velocities[name] = msg.velocity[i]

        # คำนวณจำนวนรอบของแต่ละล้อ
        for wheel, omega in wheel_velocities.items():
            self.wheel_revolutions[wheel] += (omega * delta_t) / (2 * np.pi)

        self.get_logger().info(
            f"Wheel Revolutions: FL={self.wheel_revolutions['front_left_wheel']:.3f}, "
            f"FR={self.wheel_revolutions['front_right_wheel']:.3f}, "
            f"RL={self.wheel_revolutions['rear_left_wheel']:.3f}, "
            f"RR={self.wheel_revolutions['rear_right_wheel']:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = WheelRevolutionTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


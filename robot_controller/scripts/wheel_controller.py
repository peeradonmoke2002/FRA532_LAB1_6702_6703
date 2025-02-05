#!/usr/bin/python3

from my_controller.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class WheelController(Node):
    def __init__(self):
        super().__init__('wheel_controller')
        self.wheel_pib = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)
        self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        wheel_msg = Float64MultiArray()
        # wheel_msg.data = [0.0, 0.0, 3.0, 0.0, 3.0, 0.0] #left, right
        wheel_msg.data = [0.0, 0.0]
        self.wheel_pib.publish(wheel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WheelController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

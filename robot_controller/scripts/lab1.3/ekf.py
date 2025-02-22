import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GPSEmulator(Node):
    def __init__(self):
        super().__init__('gps_emulator')
        self.publisher = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.timer = self.create_timer(1.0, self.publish_fake_gps)

    def publish_fake_gps(self):
        msg = NavSatFix()
        # Here you can set your base GPS values (latitude, longitude, altitude) and add some noise
        msg.latitude = 37.4275 + np.random.normal(0, 0.0001)
        msg.longitude = -122.1697 + np.random.normal(0, 0.0001)
        msg.altitude = 30.0 + np.random.normal(0, 0.1)
        self.publisher.publish(msg)
        self.get_logger().info(f"Fake GPS: Lat {msg.latitude}, Lon {msg.longitude}")

def main(args=None):
    rclpy.init(args=args)
    node = GPSEmulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

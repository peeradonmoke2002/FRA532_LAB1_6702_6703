#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
import os
import csv
from ament_index_python.packages import get_package_share_directory

class PathDataRecord(Node):
    def __init__(self):
        super().__init__('path_data_record')

        # Subscribe to joint states (to collect wheel and steering data).
        self.sub_joint_states = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Subscribe to Gazebo model states.
        self.sub_model_states = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )

        # Data storage.
        # Each entry: (timestamp, v_rl, v_rr, delta_fl, delta_fr)
        self.joint_states_data = []
        # Each entry: (timestamp, (x, y))
        self.model_states_data = []

        # Define wheel radius (meters) for converting wheel velocities.
        self.wheel_radius = 0.045

        # Record start time.
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # Timer to periodically save collected data (every 30 seconds here).
        self.save_timer = self.create_timer(0.01, self.save_data_to_file)

    def get_elapsed_time(self):
        return self.get_clock().now().nanoseconds / 1e9 - self.start_time

    def joint_state_callback(self, msg):
        timestamp = self.get_elapsed_time()
        # Check that the required joints exist.
        if ('rear_left_wheel' in msg.name and 'rear_right_wheel' in msg.name and
            'front_left_steering' in msg.name and 'front_right_steering' in msg.name):

            left_wheel_index = msg.name.index('rear_left_wheel')
            right_wheel_index = msg.name.index('rear_right_wheel')
            left_steering_index = msg.name.index('front_left_steering')
            right_steering_index = msg.name.index('front_right_steering')  # corrected

            # Convert wheel velocities (assumed in rad/s) to linear velocities.
            v_rl = msg.velocity[left_wheel_index] * self.wheel_radius
            v_rr = msg.velocity[right_wheel_index] * self.wheel_radius

            # Extract measured steering angles.
            delta_fl = msg.position[left_steering_index]
            delta_fr = msg.position[right_steering_index]

            self.get_logger().info(
                f"Joint states at {timestamp:.2f}s: Rear Wheel Velocities: Left: {v_rl:.2f}, Right: {v_rr:.2f} | "
                f"Steering Angles: Front Left: {delta_fl:.2f}, Front Right: {delta_fr:.2f}"
            )

            # Save the joint states data.
            self.joint_states_data.append((timestamp, v_rl, v_rr, delta_fl, delta_fr))
        else:
            self.get_logger().warn("Required joints not found in JointState message.")

    def model_states_callback(self, msg):
        timestamp = self.get_elapsed_time()
        # Collect data only for the 'limo' model.
        if "limo" in msg.name:
            index = msg.name.index("limo")
            pos = msg.pose[index].position
            self.model_states_data.append((timestamp, (pos.x, pos.y)))
            self.get_logger().debug(f"Model state at {timestamp:.2f}s: ({pos.x}, {pos.y})")

    def save_data_to_file(self):
        """Save the collected data to CSV files for later plotting."""
        path_tracking_package = get_package_share_directory("path_tracking")
        save_dir = f'{path_tracking_package}/path_data/collect_data'
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        # Save joint states data.
        joint_states_file = os.path.join(save_dir, 'joint_states_data.csv')
        with open(joint_states_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['time', 'v_rl', 'v_rr', 'delta_fl', 'delta_fr'])
            for entry in self.joint_states_data:
                writer.writerow(entry)
        
        # Save model states data.
        model_states_file = os.path.join(save_dir, 'model_states_data.csv')
        with open(model_states_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['time', 'x', 'y'])
            for timestamp, (x, y) in self.model_states_data:
                writer.writerow([timestamp, x, y])
        
        self.get_logger().info("Data saved to CSV files.")


def main(args=None):
    rclpy.init(args=args)
    node = PathDataRecord()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("DataCollector node shutting down.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

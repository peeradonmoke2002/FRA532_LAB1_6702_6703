#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from rclpy.duration import Duration
import tf_transformations
from gazebo_msgs.msg import ModelStates
import math
import yaml
import numpy as np
import matplotlib.pyplot as plt

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        
        # Publisher for steering commands (JointTrajectory for steering joints)
        self.pub_steering = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_position_controller/joint_trajectory', 
            10
        )
        # Publisher for wheel speed commands (using Float64MultiArray)
        self.pub_wheel_spd = self.create_publisher(
            Float64MultiArray, 
            '/velocity_controllers/commands', 
            10
        )
        
        # Subscriber for the robot pose from Gazebo
        self.subscription_gazebo = self.create_subscription(
            ModelStates, 
            '/gazebo/model_states', 
            self.model_states_callback, 
            10
        )
        
        # Timer to run the pure pursuit control loop
        self.timer = self.create_timer(0.01, self.pure_pursuit_control)

        # Load waypoints from a YAML file and convert to a NumPy array.
        wp = self.load_waypoints(
            '/home/peeradon/FRA532_MobileRobot/src/FRA532_LAB1_6702_6703/path_controller/config/path.yaml'
        )
        wp = np.array(wp)  # Convert list to NumPy array
        # Assuming each waypoint is (x, y, yaw); take only x and y.
        self.waypoints = wp[:, 0:2]

        # Parameters
        self.current_index = 0
        self.lookahead_distance = 1.0  # meters
        self.position = (0.0, 0.0)
        self.yaw = 0.0
        self.theta = 0.0     # Robot heading (from tf_transformations)
        self.x = 0.0         # Current x position
        self.y = 0.0         # Current y position

        # For tracking the actual path and previous position (for waypoint advance check)
        self.robot_path = []  # List to store (x, y) positions
        self.prev_position = None

        # Robot parameters (repeated here for clarity; values must match above)
        self.wheel_base = 0.2         # Distance between front and rear axles (meters)
        self.track_width = 0.14       # Distance between left and right wheels (meters)
        self.max_steering_angle = 0.523598767  # 30 degrees in radians
        self.wheel_radius = 0.045     # Rear wheel radius (meters)

        # Set up real-time plotting.
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 6))

    def load_waypoints(self, file_path):
        """Load waypoints from a YAML file."""
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
        # Expect each entry to have 'x', 'y', and optionally 'yaw'
        waypoints = [(wp['x'], wp['y'], wp.get('yaw', 0.0)) for wp in data]
        self.get_logger().info(f"Loaded {len(waypoints)} waypoints.")
        return waypoints

    def model_states_callback(self, msg):
        """Update the robot's position and orientation from Gazebo ModelStates."""
        if "limo" in msg.name:
            index = msg.name.index("limo")
            pose = msg.pose[index]
            self.position = (pose.position.x, pose.position.y)
            orientation_q = pose.orientation
            # Convert quaternion to yaw (heading)
            siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
            cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
            self.yaw = math.atan2(siny_cosp, cosy_cosp)
            # Also update theta using tf_transformations
            _, _, self.theta = tf_transformations.euler_from_quaternion([
                orientation_q.x,
                orientation_q.y,
                orientation_q.z,
                orientation_q.w
            ])
        else:
            self.get_logger().warn("Model 'limo' not found in ModelStates.")

    def publish_steering(self, steering: float):
        """
        Publish a steering command as a JointTrajectory message.
        The same steering angle is applied to both front steering joints.
        """
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['front_left_steering', 'front_right_steering']
        
        point = JointTrajectoryPoint()
        point.positions = [float(steering), float(steering)]
        # Set a small time-from-start, e.g., 0.1 seconds.
        point.time_from_start = Duration(seconds=0.1).to_msg()
        
        traj_msg.points.append(point)
        self.pub_steering.publish(traj_msg)
        self.get_logger().info(f"Published steering angle: {steering:.3f} rad")

    def pure_pursuit_control(self):
        """Compute the steering command using pure pursuit and publish commands."""
        if self.current_index >= len(self.waypoints):
            self.current_index = 0 
            self.get_logger().info("Reached final waypoint. Restarting path.")
            return
        
        # Update robot's current position.
        self.x = self.position[0]
        self.y = self.position[1]
        # Append current position to the path for plotting, ignoring the default (0,0)
        if (self.x, self.y) != (0.0, 0.0):
            self.robot_path.append((self.x, self.y))
        
        # Compute a lookahead point based on the robot's current heading.
        gazebo_lookahead_x = self.x + (self.lookahead_distance * math.cos(self.theta))
        gazebo_lookahead_y = self.y + (self.lookahead_distance * math.sin(self.theta))

        # Find the waypoint closest to the lookahead point.
        distances = np.sqrt(np.sum((self.waypoints - np.array([gazebo_lookahead_x, gazebo_lookahead_y])) ** 2, axis=1))
        min_dist_index = np.argmin(distances)
        wp_x = self.waypoints[min_dist_index][0]
        wp_y = self.waypoints[min_dist_index][1]

        # Transform the goal point to the vehicle's frame.
        dx = wp_x - self.x
        dy = wp_y - self.y
        distance = math.sqrt(dx**2 + dy**2)
        
        # If the waypoint is within the lookahead distance, only advance if the robot is actually moving.
        if distance < self.lookahead_distance:
            if self.prev_position is not None:
                movement = math.sqrt((self.x - self.prev_position[0])**2 + (self.y - self.prev_position[1])**2)
                if movement > 0.01:
                    self.get_logger().info(f"Reached waypoint {self.current_index + 1}")
                    self.current_index += 1
                    self.prev_position = (self.x, self.y)
                    return
            else:
                self.prev_position = (self.x, self.y)
        
        # Transform (dx, dy) into the robot's local coordinate frame.
        transform_x = dx * math.cos(self.theta) + dy * math.sin(self.theta)
        transform_y = -dx * math.sin(self.theta) + dy * math.cos(self.theta)

        # Calculate curvature using the pure pursuit formula:
        #   curvature = 2 * y_local / L^2, where y_local = transform_y and L = distance.
        curvature = 0.0
        if distance != 0:
            curvature = 2 * transform_y / (distance**2)
        # Compute the steering angle using a bicycle model.
        steering_angle = math.atan(self.wheel_base * curvature)
        # Optionally, apply a gain if desired (gain commented out below).
        # K = 0.6
        # steering_angle *= K

        
        # Publish the computed steering command.
        self.publish_steering(steering_angle)

        # Publish the forward wheel speed command using Float64MultiArray.
        forward_velocity = min(0.5, distance)/self.wheel_radius
        speed_msg = Float64MultiArray()
        speed_msg.data = [forward_velocity, forward_velocity]
        self.pub_wheel_spd.publish(speed_msg)

        # Update the real-time plot.
        # self.update_plot()

    def update_plot(self):
        """Update real-time plot of planned and actual paths."""
        self.ax.clear()
        # Plot planned path.
        self.ax.plot(self.waypoints[:, 0], self.waypoints[:, 1], 'go-', label="Planned Path")
        # Plot actual path if available.
        if len(self.robot_path) > 0:
            robot_path = np.array(self.robot_path)
            self.ax.plot(robot_path[:, 0], robot_path[:, 1], 'r.-', label="Actual Path")
            # Mark the current position (last point in the path).
            self.ax.scatter(robot_path[-1, 0], robot_path[-1, 1], c='purple', marker='x', label="Current Position")
        self.ax.set_title("Pure Pursuit Controller")
        self.ax.legend()
        plt.draw()
        plt.pause(0.1)
        

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Pure Pursuit node.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

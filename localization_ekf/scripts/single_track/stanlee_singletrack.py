#!/usr/bin/python3

import os
import math
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry  # Only used for publishing path and receiving odometry
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler, euler_from_quaternion

class StanleyNode(Node):
    def __init__(self):
        super().__init__('stanley_node')
        
        # Subscribe to Odometry messages to get robot state
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        # Timer for running the controller periodically (20 Hz control loop)
        self.create_timer(0.05, self.timer_callback)

        # Publishers for steering and velocity commands
        self.steering_pub = self.create_publisher(
            Float64MultiArray,
            "/position_controllers/commands",
            10
        )
        self.velocity_pub = self.create_publisher(
            Float64MultiArray,
            "/velocity_controllers/commands",
            10
        )
        # Publisher for path (for visualization)
        self.path_pub = self.create_publisher(Path, "/path", 10)

        # Load the path from YAML; each row: [x, y]
        self.path = self.load_path("path2.yaml")
        if self.path is not None:
            self.get_logger().info(f"Loaded path with {self.path.shape[0]} points.")
        else:
            self.get_logger().error("Failed to load path!")
        
        # Compute yaw for each waypoint from differences (for path tangent)
        self.path_yaw = self.compute_path_yaw(self.path)
        # Combine into a full path: [x, y, yaw]
        self.path_full = np.hstack((self.path, self.path_yaw.reshape(-1, 1)))
        self.publish_path()

        # Vehicle parameters
        self.L = 1.0   # Front axle offset [m] used in Stanley control
        self.k = 5.0   # Stanley gain
        self.speed = 2.0  # Constant speed [m/s]
        self.lastFoundIndex = 0  # Last target waypoint index

        # State variables updated from Odometry
        self.currentPos = [0.0, 0.0]
        self.currentHeading = 0.0  # in degrees

        # Storage for the robot state (from Odometry)
        self.robot_pose = None

    def load_path(self, filename):
        """
        Loads a YAML file containing a list of points.
        Each point must have keys 'x' and 'y'.
        Returns a NumPy array of shape (n, 2), where each row is [x, y].
        """
        if not os.path.isabs(filename):
            ros_workspace = os.getenv("ROS_WORKSPACE")
            if ros_workspace is None:
                script_dir = os.path.dirname(os.path.realpath(__file__))
                ros_workspace = script_dir.split('/src/')[0]
            filename = os.path.join(ros_workspace, "src", "FRA532_LAB1_6702_6703", "path_tracking", "path_data", filename)
        try:
            with open(filename, 'r') as file:
                data = yaml.safe_load(file)
            return np.array([(point['x'], point['y']) for point in data['path']])
        except Exception as e:
            self.get_logger().error(f"Error loading path: {e}")
            return None

    def compute_path_yaw(self, path):
        """
        Computes yaw (in radians) for each waypoint based on the difference to the next.
        For the last waypoint, the yaw is copied from the previous one.
        """
        n = path.shape[0]
        yaws = np.zeros(n)
        for i in range(n - 1):
            dx = path[i+1, 0] - path[i, 0]
            dy = path[i+1, 1] - path[i, 1]
            yaws[i] = math.atan2(dy, dx)
        yaws[-1] = yaws[-2] if n > 1 else 0.0
        return yaws

    def publish_path(self):
        """
        Publishes the full path as a nav_msgs/Path message.
        """
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for point in self.path_full:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]
            pose_stamped.pose.position.z = 0.0
            q = quaternion_from_euler(0, 0, point[2])
            pose_stamped.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            path_msg.poses.append(pose_stamped)
        self.path_pub.publish(path_msg)

    def normalize_angle(self, angle):
        """Normalize angle to the range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def stanley_controller_step(self, cx, cy, cyaw, currentPos, currentHeading, last_target_idx, speed):
        """
        Stanley controller step calculation.
        
        Parameters:
            cx, cy, cyaw: arrays of path x, y, and yaw values.
            currentPos: [x, y] current position.
            currentHeading: current heading in degrees.
            last_target_idx: index of the last target point.
            speed: current speed (m/s)
        
        Returns:
            delta: computed steering angle (rad)
            target_idx: updated target index
            error_front_axle: cross-track error
        """
        # Convert current heading to radians
        yaw = math.radians(currentHeading)
        # Compute front axle position
        fx = currentPos[0] + self.L * math.cos(yaw)
        fy = currentPos[1] + self.L * math.sin(yaw)
        
        # Compute distances from front axle to each waypoint
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]
        d = np.hypot(dx, dy)
        target_idx = int(np.argmin(d))
        
        # Compute cross-track error (projection on the vector perpendicular to heading)
        error_x = cx[target_idx] - fx
        error_y = cy[target_idx] - fy
        error_front_axle = error_x * (-math.sin(yaw)) + error_y * math.cos(yaw)
        
        # Compute heading error between the path tangent and vehicle heading
        theta_e = self.normalize_angle(cyaw[target_idx] - yaw)
        # Cross-track correction term
        theta_d = math.atan2(self.k * error_front_axle, speed)
        
        # Total steering command
        delta = theta_e + theta_d
        
        return delta, target_idx, error_front_axle

    def stanley_controller(self):
        """
        Computes the Stanley control action and commands the vehicle.
        """
        cx = self.path_full[:, 0]
        cy = self.path_full[:, 1]
        cyaw = self.path_full[:, 2]
        
        if self.robot_pose is None:
            self.get_logger().warn("No robot state received yet.")
            return
        
        # Extract current state from self.robot_pose (from Odometry)
        # เพิ่ม offset เล็กน้อยถ้าจำเป็น
        x = self.robot_pose.position.x + 0.10
        y = self.robot_pose.position.y
        orientation_q = self.robot_pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.currentPos = [x, y]
        self.currentHeading = math.degrees(yaw)
        
        delta, target_idx, cte = self.stanley_controller_step(
            cx, cy, cyaw, self.currentPos, self.currentHeading, self.lastFoundIndex, self.speed)
        
        # Ensure target index does not go backward
        if target_idx < self.lastFoundIndex:
            target_idx = self.lastFoundIndex
        self.lastFoundIndex = target_idx
        
        self.get_logger().info(
            f"Steering Angle: {delta:.2f} rad, TargetIdx: {target_idx}, Cross-Track Error: {cte:.2f}"
        )
        
        self.cmd_vel_steering(self.speed, delta)

    def cmd_vel_steering(self, vx, steering_angle):
        """
        Converts desired steering angle and speed into commands.
        Publishes:
          - Steering command as a Float64MultiArray (position control)
          - Velocity command as a Float64MultiArray (velocity control)
        """
        # Limit steering angle to ±10°
        angle_max = math.radians(10.0)
        angle_min = -math.radians(10.0)
        steering_angle = np.clip(steering_angle, angle_min, angle_max)
        
        # Convert linear velocity to wheel speed using the wheel radius.
        wheel_speed = vx / 0.3  # assuming wheel_radius = 0.3 [m]
        
        # Publish steering command (same command for both front steering joints)
        self.set_steering_angle(steering_angle, steering_angle)
        # Publish velocity command (same speed for both rear wheels)
        self.set_velocity(wheel_speed, wheel_speed)
    
    def timer_callback(self):
        """
        Periodic timer callback that republishes the path and runs the Stanley controller.
        """
        self.publish_path()
        self.stanley_controller()

    def odom_callback(self, msg: Odometry):
        """
        Callback for /odom messages.
        Updates the robot_pose from Odometry.
        """
        self.robot_pose = msg.pose.pose

    def set_steering_angle(self, left_angle, right_angle):
        msg = Float64MultiArray()
        msg.data = [float(left_angle), float(right_angle)]
        self.steering_pub.publish(msg)
    
    def set_velocity(self, left_speed, right_speed):
        msg = Float64MultiArray()
        msg.data = [float(left_speed), float(right_speed)]
        self.velocity_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StanleyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import matplotlib.pyplot as plt
import numpy as np
from tf_transformations import euler_from_quaternion


class OdomGazeboPlotter(Node):
    def __init__(self):
        super().__init__('odom_gazebo_plotter')

        # ✅ Subscribe to /odom
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.get_logger().info("✅ Subscribed to /odom")

        # ✅ Subscribe to /gazebo/model_states
        self.gazebo_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.gazebo_callback,
            10)
        self.get_logger().info("✅ Subscribed to /gazebo/model_states")

        # ✅ Data storage for plotting
        self.odom_x, self.odom_y = [], []
        self.gazebo_x, self.gazebo_y = [], []
        self.timer = self.create_timer(0.05, self.compute_rms_error)

        
        # ✅ Setup real-time plot
        plt.ion()  # Enable interactive mode
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.odom_plot, = self.ax.plot([], [], 'bo-', label="Odom")  # Blue line
        self.gazebo_plot, = self.ax.plot([], [], 'ro-', label="Gazebo")  # Red line
        self.ax.legend()
        self.ax.set_xlabel("X Position")
        self.ax.set_ylabel("Y Position")
        self.ax.set_title("Odometry vs Gazebo Position | single_model_noslip_with_traction_control")
        self.ax.grid(True)

    def odom_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        # ✅ Convert quaternion to Euler angles
        quaternion = (q.x, q.y, q.z, q.w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)  # Extract yaw rotation

        # ✅ Apply TF rotation transformation
        x_transformed = x
        y_transformed = y



        self.get_logger().info(f"📡 Odometry → X: {x_transformed:.3f}, Y: {y_transformed:.3f}")

        self.odom_x.append(x_transformed)
        self.odom_y.append(y_transformed)
        self.update_plot

    def compute_rms_error(self):
        min_length = min(len(self.odom_x), len(self.gazebo_x))
        
        if min_length == 0:
            self.get_logger().warn("❌ Not enough data for RMS calculation!")
            return
        
        error_x = np.array(self.odom_x[:min_length]) - np.array(self.gazebo_x[:min_length])
        error_y = np.array(self.odom_y[:min_length]) - np.array(self.gazebo_y[:min_length])

        rms_x = np.sqrt(np.mean(error_x**2))
        rms_y = np.sqrt(np.mean(error_y**2))

        self.get_logger().info(f"📊 RMS Error → X: {rms_x:.3f} m, Y: {rms_y:.3f} m")


    def gazebo_callback(self, msg):

        robot_name = "limo"  

        if robot_name in msg.name:
            index = msg.name.index(robot_name)
            x = msg.pose[index].position.x
            y = msg.pose[index].position.y
            self.gazebo_x.append(x)
            self.gazebo_y.append(y)
            self.update_plot()
        else:
            self.get_logger().warn("🚨 Robot model not found in /gazebo/model_states!")

    def update_plot(self):
        """Update the real-time plot with total RMS error using Mean Square Root (MSR)"""
        self.odom_plot.set_data(self.odom_x, self.odom_y)
        self.gazebo_plot.set_data(self.gazebo_x, self.gazebo_y)

        # ✅ Compute Total RMS Error using Mean Square Root (MSR)
        min_length = min(len(self.odom_x), len(self.gazebo_x))
        if min_length > 0:
            error_x = np.array(self.odom_x[:min_length]) - np.array(self.gazebo_x[:min_length])
            error_y = np.array(self.odom_y[:min_length]) - np.array(self.gazebo_y[:min_length])

            # Individual RMS errors
            rms_x = np.sqrt(np.mean(error_x**2))
            rms_y = np.sqrt(np.mean(error_y**2))

            # ✅ Compute Total RMS Error (Mean Square Root of X and Y)
            total_rms_error = np.sqrt(np.mean(error_x**2 + error_y**2))

            # ✅ Display RMS error on the graph title
            self.ax.set_title(f"Odometry vs Gazebo Position\n📊 RMS Error → X: {rms_x:.3f} m, Y: {rms_y:.3f} m, Total: {total_rms_error:.3f} m")
        
        self.ax.relim()
        self.ax.autoscale_view()

        plt.draw()
        plt.pause(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = OdomGazeboPlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

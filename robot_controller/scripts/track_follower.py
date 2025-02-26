#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge

# Constants
LINEAR_SPEED = 0.5  # Adjust based on your desired linear speed
KP = 0.5  # Adjust based on your desired proportional gain

class TrackFollower(Node):
    def __init__(self):
        super().__init__('track_follower')
        self.subscription = self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_publisher = self.create_publisher(Image, 'debug_image', 10)
        self.twist_msg = Twist()
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert the image message to an OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert the image to HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the color range for the white track
        lower_white = (0, 0, 200)
        upper_white = (255, 30, 255)

        # Threshold the image to get only the white track
        mask = cv2.inRange(hsv_image, lower_white, upper_white)

        # Find the contours of the white track
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # Find the largest contour (assumed to be the track)
            track_contour = max(contours, key=cv2.contourArea)

            # Calculate the moments of the track contour
            moments = cv2.moments(track_contour)
            if moments["m00"] > 0:
                # Calculate the centroid of the track
                cx = int(moments["m10"] / moments["m00"])
                cy = int(moments["m01"] / moments["m00"])

                # Calculate the error
                error = cx - cv_image.shape[1] // 2

                # Adjust the robot's velocities based on the error
                self.twist_msg.linear.x = LINEAR_SPEED
                self.twist_msg.angular.z = -KP * error / 100

                # Publish the velocity command
                self.publisher.publish(self.twist_msg)

                # Draw the contour and centroid on the image
                cv2.drawContours(cv_image, [track_contour], 0, (0, 255, 0), 2)
                cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)

        # Publish the debug image
        debug_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.debug_publisher.publish(debug_image_msg)

def main(args=None):
    rclpy.init(args=args)
    track_follower = TrackFollower()
    rclpy.spin(track_follower)
    track_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
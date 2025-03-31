#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

class SideCam:
    def __init__(self):
        self.bridge = CvBridge()

        self.right_image = None
        self.left_image = None

        self.right_image_sub = rospy.Subscriber("/B1/rrbot/camera_right/image_right_raw", Image, self.right_image_callback)
        self.left_image_sub = rospy.Subscriber("/B1/rrbot/camera_left/image_left_raw", Image, self.left_image_callback)

    def right_image_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # Convert to OpenCV format

    def left_image_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # Convert to OpenCV format

    def get_image(self):
        return self.right_image, self.left_image
    
    def process_image(self):
        """Process and return the latest image from the requested camera."""
        image_right, image_left = self.get_image()  # Get the latest image

        if right_image is None or left_image is None:
            rospy.logwarn("No image from cam.")
            return None

        # Gonna process these differently
        processed_image_right = cv2.cvtColor(image_right, cv2.COLOR_BGR2GRAY)
        processed_image_left = cv2.cvtColor(image_left, cv2.COLOR_BGR2GRAY)

        return processed_image_right, processed_image_left

if __name__ == "__main__":
    cam = SideCam()
    rospy.spin

    cv2.waitKey(0)
    cv2.destroyAllWindows()
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
        rospy.init_node("side_cam_listener", anonymous=True)
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
    
if __name__ == "__main__":
    cam = SideCam()
    rospy.spin()
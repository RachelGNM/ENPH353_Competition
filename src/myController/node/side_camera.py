#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

"""
@file side_camera.py

@brief check the side cameras
"""

class SideCam:
    def __init__(self):
        self.bridge = CvBridge()

        self.right_image = None
        self.left_image = None

        self.lower_blue = np.array([90, 90, 50])  # Lower bound of blue
        self.upper_blue = np.array([130, 255, 255])  # Upper bound of blue

        self.right_image_sub = rospy.Subscriber("/B1/rrbot/camera_right/image_right_raw", Image, self.right_image_callback)
        self.left_image_sub = rospy.Subscriber("/B1/rrbot/camera_left/image_left_raw", Image, self.left_image_callback)

    def right_image_callback(self, msg):
        """
        @brief get image from right camera

        @warn image might be None
        
        @todo implement throw exception if image is None
        """
        self.right_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # Convert to OpenCV format

    def left_image_callback(self, msg):
        """
        @brief get image from right camera

        @warn image might be None
        
        @todo implement throw exception if image is None
        """
        self.left_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # Convert to OpenCV format

    def get_image(self):
        """
        @brief get the images from both cameras

        @returns right and left camera feeds
        """
        return self.right_image, self.left_image
    
    def process_image(self, left):
        """
        @brief process for blue rectangle and return the latest image from the requested camera
        
        @param left whether the image being processed should be the left one

        @return whether a blue rectangle is found, processed image with drawn box around clueboard
        """
        image_right, image_left = self.get_image()  # Get the latest image

        if left:
            image = image_left
        else:
            image = image_right

        if image is None:
            rospy.logwarn("No image from cam.")
            return None

        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create blue mask
        blue_mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)

        # Eliminate top 40% of the image from the mask
        height = blue_mask.shape[0]
        cutoff = int(0.4 * height)
        blue_mask[0:cutoff, :] = 0  # Set top 40% to black (mask off)

        # Apply the modified mask
        result = cv2.bitwise_and(image, image, mask=blue_mask)

        # Find contours in the blue mask
        contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Iterate over the contours
        for contour in contours:
            area = cv2.contourArea(contour)

            # Threshold area to filter out small contours (board area)
            if area > 5000:  # Adjust threshold based on the size of the board
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green bounding box
                return True, image

        return False, image

if __name__ == "__main__":
    cam = SideCam()
    rospy.spin

    cv2.waitKey(0)
    cv2.destroyAllWindows()
#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class MotionDetector:
    def __init__(self):
        # Initialize the background subtractor (MOG2)
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(history=300, varThreshold=100, detectShadows=True)

    def detect_movement(self, image, zone):
        # height, width, _ = image.shape
        # image = image[height // 4 : 3 * height // 4, width // 4 : 3 * width // 4]

        # rospy.loginfo("Detecting movement using background subtraction")

        # # Apply the background subtractor
        # fg_mask = self.bg_subtractor.apply(image)

        # # Optional: Remove noise using morphological operations
        # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        # fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel)

        # # Show the foreground mask for debugging
        # cv2.imshow("Foreground Mask", fg_mask)

        # # Find contours of moving objects
        # contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # # Return True if movement is detected
        # if len(contours) > 0:
        #     rospy.loginfo("Movement detected!")
        #     return True
        # else:
        #     rospy.loginfo("No movement detected.")
        #     return False

        height, width, _ = image.shape
        image = image[height // 4:3* height // 4, :]
        image = cv2.GaussianBlur(image, (5, 5), 0)
        

        expected_contours = 1

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        height_after, _, _ = image.shape
        height_threshold = height_after // 2
        if zone == 1:
            #rospy.loginfo("Checking for red line")
            # Define HSV range for red color (two ranges needed for red hue wrap-around)
            lower_red1 = np.array([0, 100, 100])   # Lower range of red
            upper_red1 = np.array([10, 255, 255])  
            lower_red2 = np.array([170, 100, 100])  # Upper range of red
            upper_red2 = np.array([180, 255, 255])  

            # Create masks for red regions
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            red_mask = cv2.bitwise_or(mask1, mask2)

            # Convert all previously white pixels (255) to black (0)
            processed_image = cv2.bitwise_and(image, image, mask=red_mask)

            # Extract only the red channel
            image = processed_image[:, :, 2]  # Extract the R channel from BGR

            contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            cv2.imshow("Red", image)

            expected_contours = 1
        if len(contours) > expected_contours:
            rospy.loginfo("More than expected contours")
            rospy.loginfo(f"Contours: {len(contours)}")
        else:
            rospy.loginfo("No movement")
        return len(contours) > expected_contours

    

    

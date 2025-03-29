#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def line_following(binary_image):
    # Find contours
    contours, _ = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Process only if at least one contour is found

    if contours:
        # Find the largest contour (assuming it's the main object)
        c = max(contours, key=cv2.contourArea)
        # Compute moments
        M = cv2.moments(c)
        # Compute centroid coordinates
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            #Centroid: ({cx}, {cy})
            error = cx - width / 2

            turn = self.Kp * error
            move.angular.z = turn
            if abs(turn) < 0.5:
                move.linear.x = 0.2
            elif abs(turn) < 0.7:
                move.linear.x = 0.1
            else:
                move.angular.z = 0.3 * turn
                move.linear.x = 0.08
            cmd_vel_pub.publish(move)

        else:
            move.angular.z = 1.0
            move.linear.x = 0
    else:
        move.angular.z = 1.0
        move.linear.x = 0
#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RoadProcessing:
    def road_binarize(image_feed, zone):
        if zone < 2:
            threshold = 90
            wall_threshold = 80
            #Convert frame to binary
            blur_frame = cv2.GaussianBlur(image_feed, (5, 5), 0)
            # gray_frame = cv2.cvtColor(blur_frame, cv2.COLOR_BGR2GRAY)
            gray_frame = blur_frame[:,:,1]
            frame_no_wall = np.where((gray_frame < wall_threshold), 255, gray_frame)
            _, img_bin = cv2.threshold(frame_no_wall, threshold, 255, cv2.THRESH_BINARY)
            # Get image dimensions
            height, width = img_bin.shape
            # Turn the top section white
            img_bin[:2 * height // 3, :] = [255]
            return img_bin

    def line_follow(binary_image, Kp, Kd, last_error):
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

                turn = Kp * error + Kd * (error - last_error) / 2
                last_error = error

                move.angular.z = turn
                if abs(turn) < 0.5:
                    move.linear.x = 1
                elif abs(turn) < 0.7:
                    move.linear.x = 0.5
                else:
                    move.angular.z = turn
                    move.linear.x = 0.08
                cmd_vel_pub.publish(move)

            else:
                move.linear.x = -0.2
                cmd_vel_pub.publish(move)
        else:
            move.linear.x = -0.2
            cmd_vel_pub.publish(move)
        return last_error


    def stopping_point(image_feed, zone):
        #TODO: if the line (either red or fuchsia) is found return true
        #TODO: figure out how to binarize image to show only red or fuchsia line without including white line
        height, width, _ = image_feed.shape
        line_image = image_feed[3* height // 4:, :]
        hsv = cv2.cvtColor(line_image, cv2.COLOR_BGR2HSV)

        height_after, _, _ = line_image.shape
        height_threshold = height_after // 2
        if zone < 2:
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
            processed_image = cv2.bitwise_and(line_image, line_image, mask=red_mask)

            # Extract only the red channel
            line_image = processed_image[:, :, 2]  # Extract the R channel from BGR
        else:
            # Define HSV range for fuchsia/magenta
            lower_fuchsia = np.array([140, 100, 100])  # Lower bound
            upper_fuchsia = np.array([165, 255, 255])  # Upper bound

            # Create a mask
            fuchsia_mask = cv2.inRange(hsv, lower_fuchsia, upper_fuchsia)

            # Remove any white pixels (255) from previous processing
            processed_image = cv2.bitwise_and(line_image, line_image, mask=fuchsia_mask)

            # Extract red and blue channels
            red_channel = processed_image[:, :, 2]
            blue_channel = processed_image[:, :, 0]

            # Combine red and blue channels
            line_image = cv2.addWeighted(red_channel, 0.5, blue_channel, 0.5, 0)
        #Check if the bottom area of image is mostly white
        
        # Apply thresholding to detect white areas
        _, binary = cv2.threshold(line_image, 200, 255, cv2.THRESH_BINARY)  # Adjust 200 if needed
        
        # Find white pixels
        white_pixels = np.where(binary == 255)

        if len(white_pixels[0]) == 0:
            return False  # No white pixels found
        
        # Get the highest white pixel (smallest y-value)
        topmost_white_pixel = np.min(white_pixels[0])  # y-coordinate

        # Return True if the white line reaches the threshold
        return topmost_white_pixel <= height_threshold



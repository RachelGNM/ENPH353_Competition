#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RoadProcessing:
    def road_binarize(self, image_feed, zone):
        if zone <= 4:
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
        # else:



    def stopping_point(self, image_feed, zone):
        #TODO: if the line (either red or fuchsia) is found return true
        #TODO: figure out how to binarize image to show only red or fuchsia line without including white line
        height, width, _ = image_feed.shape
        line_image = image_feed[3* height // 4:, :]
        hsv = cv2.cvtColor(line_image, cv2.COLOR_BGR2HSV)

        height_after, _, _ = line_image.shape
        height_threshold = height_after // 2
        if zone < 2:
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
            processed_image = cv2.bitwise_and(line_image, line_image, mask=red_mask)

            # Extract only the red channel
            line_image = processed_image[:, :, 2]  # Extract the R channel from BGR

            zone = 1
        elif zone > 3: #looking for fuchsia lines
            rospy.loginfo("Checking for fuchsia line")
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
            zone += 1
        #TODO: define for zone 2 --> zone 3 (aka for the car, must just wait for clue = 3)
        #Check if the bottom area of image is mostly white
        
        cv2.imshow("image", line_image)
        cv2.waitKey(1)

        # Apply thresholding to detect white areas
        _, binary = cv2.threshold(line_image, 200, 255, cv2.THRESH_BINARY)  # Adjust 200 if needed
        
        # Find white pixels
        white_pixels = np.where(binary == 255)

        if len(white_pixels[0]) == 0:
            return zone, False  # No white pixels found
        
        # Get the highest white pixel (smallest y-value)
        topmost_white_pixel = np.min(white_pixels[0])  # y-coordinate

        # Return True if the white line reaches the threshold
        return zone, (topmost_white_pixel >= height_threshold and self.detect_horizontal_line(binary))


    def detect_horizontal_line(self, binary_image):
        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            if len(contour) > 10:  # Ignore small noise
                # Fit a line to the contour
                [vx, vy, x0, y0] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)

                # Compute angle in degrees
                angle = np.arctan2(vy, vx) * 180 / np.pi

                # Check if the line is nearly horizontal (angle near 0 or 180 degrees)
                if abs(angle) < 10 or abs(angle - 180) < 10:
                    return True  # Crosswalk detected in horizontal orientation

        return False  # No horizontal crosswalk detected

    def detect_sign(self, image, sign_number):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define HSV range for blue
        lower_blue = np.array([110, 150, 200])  # Lower bound for blue
        upper_blue = np.array([130, 255, 255])  # Upper bound for blue

        # Create mask
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Convert all previously white pixels (255) to black (0)
        processed_image = cv2.bitwise_and(image, image, mask=blue_mask)

        # Check if any blue pixels are found
        if np.any(blue_mask):
            return sign_number + 1, True  # Blue detected
        else:
            return sign_number, False  # No blue detected

    def detect_movement(self, prev_image, image):
        rospy.loginfo("Detecting movement")

        prev_gray = cv2.cvtColor(prev_image, cv2.COLOR_BGR2GRAY)
        curr_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        #Calculate the difference between the images
        diff = cv2.absdiff(prev_gray,curr_gray)

        _, threshold = cv2.threshold(diff,threshold_value, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        return len(contours) > 0



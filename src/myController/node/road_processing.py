#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RoadProcessing:
    def __init__(self):
        self.prev_image = None
        self.three_image = None

    def road_binarize(self, image_feed, zone):
        if zone == 4 or zone < 3:
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
        elif zone == 3:
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
            return img_bin
        else:
            return self.grass_to_line(image_feed)



    def grass_to_line(self, cv_image):
        #Convert frame to binary
        height, _, _ = cv_image.shape
        cv_image = cv_image[height // 2: 4 * height // 5,:,:]

        blur_frame = cv2.GaussianBlur(cv_image, (7, 7), 0)

        hsv = cv2.cvtColor(blur_frame,cv2.COLOR_BGR2HSV)

        # Define HSV range for grass
        lower = np.array([20, 0, 180])  # Lower bound 
        upper = np.array([40, 70, 255])  # Upper bound 

        # Create mask
        grass_mask = cv2.inRange(hsv, lower, upper)

        # Define structuring element
        erode_kernel = np.ones((5,5), np.uint8)

        # Perform erosion
        erosion = cv2.erode(grass_mask, erode_kernel, iterations = 1)

        # blur_frame = cv2.GaussianBlur(img, (5, 5), 0)
        dilate_kernel = np.ones((7,7),np.uint8)

        dilated_img = cv2.dilate(erosion, dilate_kernel, iterations=1)

         # Find contours
        contours, _ = cv2.findContours(dilated_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Create a fully white image
        final_img = np.ones_like(grass_mask) * 255

        min_contour_area = 200  # Adjust this value based on your setup
        for contour in contours:
            if cv2.contourArea(contour) > min_contour_area:
                cv2.drawContours(final_img, [contour], -1, (0), thickness=cv2.FILLED)  # Draw black lines

        final_img = cv2.dilate(final_img, dilate_kernel, iterations=1)
        cv2.imshow("Final",final_img)

        return final_img


    def stopping_point(self, image_feed, zone):
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
        elif zone > 3: #looking for fuchsia lines
            # rospy.loginfo("Checking for fuchsia line")
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

        found = (topmost_white_pixel >= height_threshold and self.detect_horizontal_line(binary))

        if found:
            if zone < 2:
                zone = 1

            elif zone > 3:
                zone += 1

        # Return True if the white line reaches the threshold
        return zone, found


    def detect_horizontal_line(self, binary_image):
        if len(binary_image.shape) == 3:
            binary_image = cv2.cvtColor(binary_image, cv2.COLOR_BGR2GRAY)

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
        height, width, _ = image.shape
        image = image[height // 2 :, :,:]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define HSV range for blue
        lower_blue = np.array([90, 90, 90])  # Lower bound for blue
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
    
    def detect_left_turn(self, image):
        height, width = image.shape

        image = image[7 * height // 16: 5 * height // 8, : width // 4]

        cv2.imshow("Left turn search", image)

        image = image[: height // 2, : width // 4]

        image = cv2.GaussianBlur(image, (5, 5), 0)
        _, image = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        prev_len = 0

        cv2.imshow("Left turn search", image)
        
        for contour in contours:
            if len(contour) > prev_len:
                prev_len = len(contour)
                rospy.loginfo(f"Largest contour: {prev_len}")

            if len(contour) > 100:  # Ignore small noise
                rospy.loginfo(f"Large contour: {len(contour)}")
                return True

        return False

    def find_intersection(self,image):
        image = self.road_binarize(image,2)
        height, width= image.shape

        image1 = image[7 * height // 16: 5 * height // 8, : width // 4]
        image2 = image[7 * height // 16: 5 * height // 8, 3* width // 4: ]

        cv2.imshow("Left turn search", image)

        image1 = image1[: height // 2, : width // 4]
        image2 = image2[: height // 2, 3* width // 4 :]
        cv2.imshow("Left", image1)
        cv2.imshow("Right",image2)

        left_road, left_road_img = self.find_intersection_contour(image1) 
        right_road, right_road_img = self.find_intersection_contour(image2)

        # cv2.imshow("Left", left_road_img)
        # cv2.imshow("Right", right_road_img)

        return zone, left_road and right_road

    def find_intersection_contour(self, image):
        if image is not None:
            image = cv2.GaussianBlur(image, (5, 5), 0)
            _, image = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY)

            contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            prev_len = 0

            # cv2.imshow("Left turn search", image)
            
            for contour in contours:
                if len(contour) > prev_len:
                    prev_len = len(contour)
                    rospy.loginfo(f"Largest contour: {prev_len}")

                if len(contour) > 100:  # Ignore small noise
                    rospy.loginfo(f"Large contour: {len(contour)}")
                    return True, image

            return False, image
        else:
            rospy.loginfo("No image received")



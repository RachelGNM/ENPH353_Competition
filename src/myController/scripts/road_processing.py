#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
"""
@file road_processing.py
@brief the RoadProcessing class reads images depending on the robot's zone and looks for areas of interest
"""

class RoadProcessing:
    def __init__(self):
        self.largest_contour = 0
        self.reach_pond = False

    def road_binarize(self, image_feed, zone):
        """
        @brief binarizes an input image to use for line following

        @param image_feed the raw image from the robot's camera
        @param zone the zone in which the robot is located

        @return the binarized image
        """
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
            height, width = gray_frame.shape
            line_image = image_feed[3* height // 4:, :]
            hsv = cv2.cvtColor(line_image, cv2.COLOR_BGR2HSV)

            height_after, _, _ = line_image.shape
            height_threshold = height_after // 2
            frame_no_wall = np.where((gray_frame < wall_threshold), 255, gray_frame)
            _, img_bin = cv2.threshold(frame_no_wall, threshold, 255, cv2.THRESH_BINARY)
            # Get image dimensions
            height, width = img_bin.shape
            return img_bin
        elif zone == 8:
            image_feed = image_feed[height //2:,:,:]
            hsv = cv2.cvtColor(image_feed, cv2.COLOR_BGR2HSV)

            # Define HSV range for blue
            lower_brown = np.array([150, 0, 0])  # Lower bound for blue
            upper_brown = np.array([255, 60, 60])  # Upper bound for blue

            # Create mask
            brown_mask = cv2.inRange(hsv, lower_brown, upper_brown)

            # Convert all previously white pixels (255) to black (0)
            processed_image = cv2.bitwise_and(image_feed, image_feed, mask=brown_mask)

            line_image = cv2.cvtColor(processed_image, cv2.COLOR_BGR2GRAY)

            _, img_bin = cv2.threshold(line_image, 10, 255, cv2.THRESH_BINARY)

            img_bin = cv2.bitwise_not(img_bin)
            return img_bin
        else:
            return self.grass_to_line(image_feed)



    def grass_to_line(self, cv_image):
        """
        @brief takes the image from the grassland and converts it to a binary image such that the line defining the road is black on a white background

        @param cv_image the raw image from the camera feed

        @return binarized image
        """
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
        # cv2.imshow("Final",final_img)

        return final_img


    def stopping_point(self, image_feed, zone):
        """
        @brief looks for a point of interest at which the robot needs to stop and then not line follow

        @param image_feed raw camera feed
        @param zone current robot's location

        @return the zone and whether a stopping point was found
        """
        line_image = image_feed
        height, width, _ = line_image.shape
        height_threshold = height // 2
        if zone < 2:
            line_image = line_image[3* height // 4:, :]
            hsv = cv2.cvtColor(line_image, cv2.COLOR_BGR2HSV)

            height_after, _, _ = line_image.shape
            height_threshold = height_after // 2
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
        elif zone == 2:
            line_image = line_image[height // 4: 3 * height // 4, :]
            hsv = cv2.cvtColor(line_image, cv2.COLOR_BGR2HSV)

            height_after, _, _ = line_image.shape
            height_threshold = height_after // 2
            
            # Define the green color range
            lower_green = np.array([35, 40, 40])   # Lower bound of green (H, S, V)
            upper_green = np.array([85, 255, 255]) # Upper bound of green (H, S, V)

            # Create the mask
            mask = cv2.inRange(hsv, lower_green, upper_green)

            # Apply mask to original image (optional)
            green_mask = cv2.bitwise_and(line_image, line_image, mask=mask)

            line_image = green_mask[:,:,1]
            # cv2.imshow("Green", line_image)

            # Apply thresholding to detect white areas
            _, binary = cv2.threshold(line_image, 100, 255, cv2.THRESH_BINARY)  # Adjust 200 if needed

            binary = binary[height_after // 3: 2 * height_after //3, width // 3: 2 * width // 3]

            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            self.largest_contour = max(len(contours), self.largest_contour)
            rospy.loginfo(f"NumCont = {len(contours)}, MaxCont = {self.largest_contour}")
            if self.largest_contour > 130 and len(contours) < 30:
                return zone, True
            else:
                return zone, False
        elif zone == 4:
            line_image = line_image[ height // 2:, :]
            hsv = cv2.cvtColor(line_image, cv2.COLOR_BGR2HSV)

            height_after, _, _ = line_image.shape
            height_threshold = height_after // 3
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
        elif zone == 5 or zone == 6:
            line_image = line_image[ height // 2:, :]
            hsv = cv2.cvtColor(line_image, cv2.COLOR_BGR2HSV)

            height_threshold = 3 * height // 5

            # Define HSV range for blue
            lower_blue = np.array([100, 0, 100])  # Lower bound for blue
            upper_blue = np.array([255, 100, 255])  # Upper bound for blue

            # Create mask
            blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

            # Convert all previously white pixels (255) to black (0)
            processed_image = cv2.bitwise_and(line_image, line_image, mask=blue_mask)

            line_image = cv2.cvtColor(processed_image, cv2.COLOR_BGR2GRAY)

            _, img_bin = cv2.threshold(line_image, 10, 255, cv2.THRESH_BINARY)

            contours, _ = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            rospy.loginfo(f"NumCont = {len(contours)}")

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                largest_area = cv2.contourArea(largest_contour)
                print(f"Largest Contour Area: {largest_area}")
                if zone == 5:
                    # Find all white pixels (nonzero pixels)
                    white_pixels = np.column_stack(np.where(img_bin == 255))
                    # Check if any white pixel reaches the last column
                    if white_pixels.size > 0 and np.max(white_pixels[:, 1]) == img_bin.shape[1] - 1 and largest_area > 8000:
                        rospy.loginfo("White pixels have reached right")
                        self.reach_pond = True
                    elif self.reach_pond and largest_area < 5:
                        self.reach_pond = False
                        return zone, True
                else:
                    if largest_area > 25000:
                        self.reach_pond = False
                        rospy.loginfo("Zone switch")
                        return zone, True
            # elif self.reach_pond and zone == 6:
            #         rospy.loginfo("Zone switch")
            #         return zone, True

            # # Find all white pixels (nonzero pixels)
            # white_pixels = np.column_stack(np.where(img_bin == 255))
            # # Check if any white pixel reaches the last column
            # if white_pixels.size > 0 and np.max(white_pixels[:, 1]) == img_bin.shape[1] - 1:
            #     # return zone, True
            #     rospy.loginfo("White pixels have reached right")
            return zone, False

        elif zone >6: #looking for fuchsia lines

            # rospy.loginfo("Looking for fuchsia line")
        
            line_image = line_image[3* height // 4:, width // 2:]
            hsv = cv2.cvtColor(line_image, cv2.COLOR_BGR2HSV)

            height_after, _, _ = line_image.shape
            height_threshold = height_after // 2
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

        if not zone == 3:
            # Apply thresholding to detect white areas
            _, binary = cv2.threshold(line_image, 200, 255, cv2.THRESH_BINARY)  # Adjust 200 if needed
            
            # Find white pixels
            white_pixels = np.where(binary == 255)

            if len(white_pixels[0]) == 0:
                return zone, False  # No white pixels found
            
            # Get the highest white pixel (smallest y-value)
            topmost_white_pixel = np.min(white_pixels[0])  # y-coordinate
            # bottommost_white_pixel = np.max(white_pixels[0])

            found = (topmost_white_pixel >= height_threshold and self.detect_horizontal_line(binary))
            # found = (bottommost_white_pixel == 0 and self.detect_horizontal_line(binary))

            if found:
                if zone == 0:
                    zone = 1

            # Return True if the white line reaches the threshold
            return zone, found
        return zone, False


    def detect_horizontal_line(self, binary_image):
        """
        @brief sees if a line is horizontal or near horizontal

        @param binary_image binarized image from stopping_point function

        @return T/F whether there is a found horizontal line
        """
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
        """
        @brief looks for the blue signs

        @param image raw camera feed
        @param sign_number number of clueboards found thus far

        @return updated number of clues seen, whether clue was seen
        """
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
        """
        @brief detects when there is a left turn after following the truck

        @param image raw camera feed

        @return whether it is time to turn left
        """
        height, width = image.shape

        image = image[7 * height // 16: 5 * height // 8, : width // 4]

        # cv2.imshow("Left turn search", image)

        image = image[: height // 2, : width // 4]

        image = cv2.GaussianBlur(image, (5, 5), 0)
        _, image = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        prev_len = 0

        # cv2.imshow("Left turn search", image)
        
        for contour in contours:
            if len(contour) > prev_len:
                prev_len = len(contour)
                rospy.loginfo(f"Largest contour: {prev_len}")

            if len(contour) > 100 and len(contour) < 160: # Ignore small noise
                rospy.loginfo(f"Large contour: {len(contour)}")
                return True

        return False

    def find_intersection(self,image):
        """
        @brief look for the intersection with the truck

        @param image raw camera feed

        @return whether there is a left road and right road seen
        """
        image = self.road_binarize(image,2)
        height, width= image.shape

        image1 = image[7 * height // 16: 5 * height // 8, : width // 4]
        image2 = image[7 * height // 16: 5 * height // 8, 3* width // 4: ]

        # cv2.imshow("Left turn search", image)

        image1 = image1[: height // 2, : width // 4]
        image2 = image2[: height // 2, 3* width // 4 :]
        # cv2.imshow("Left", image1)
        # cv2.imshow("Right",image2)

        left_road, left_road_img = self.find_intersection_contour(image1) 
        right_road, right_road_img = self.find_intersection_contour(image2)

        # cv2.imshow("Left", left_road_img)
        # cv2.imshow("Right", right_road_img)

        return left_road and right_road

    def find_intersection_contour(self, image):
        """
        @brief find the contours of a binary image and look for a large enough one to be the intersection

        @param image binarized image

        @return whether a large contour is found, processed image
        """
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



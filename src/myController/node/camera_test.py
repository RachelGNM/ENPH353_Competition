#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from road_processing import RoadProcessing

TEAM_NAME = "Smithies"
PASSWORD = "Volcan"

"""
@file camera_test.py

@brief test various image processing functions
"""

class CameraTesting:
    def __init__(self):
        rospy.init_node('camera_test', anonymous=True)

        self.timer_started = False
        self.timer_ended = False
        self.start_time = None
        # self.endpoint = 30
        
        # Publishers
        self.timer_pub = rospy.Publisher('/score_tracker', String, queue_size=1)
        # self.cmd_vel_pub = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)

        self.image_sub = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.image_callback)

        self.bridge=CvBridge()

        self.threshold = 90
        self.wall_threshold = 82
        self.last_error = 0

        self.road = RoadProcessing()
        self.prev_image = None
        self.three_image = None

        rospy.sleep(2)  # Ensure publishers are ready

        rospy.on_shutdown(self.stop_timer)  # Ensure the timer stops when script ends

    def start_timer(self):
        """
        @brief start the timer
        """
        msg = f"{TEAM_NAME},{PASSWORD},0,NA"
        rospy.loginfo(f"Starting timer: {msg}")
        self.timer_pub.publish(msg)
        self.start_time = rospy.Time.now().to_sec()
        self.timer_started = True


    def stop_timer(self):
        """
        @brief stop the timer and stop movement
        """
        if self.timer_started == True:
            msg = f"{TEAM_NAME},{PASSWORD},-1,NA"
            rospy.loginfo(f"Stopping timer: {msg}")
            self.timer_pub.publish(msg)
            self.timer_ended = True

    def image_callback(self, msg):
        """
        @brief run image processing functions
        """
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")
        cv2.imshow("Input", cv_image)

        self.main_intersection(cv_image)

        # self.test_find_blue(cv_image, True)
        # self.grass_road(cv_image)

        # # true = self.road.detect_movement(self.prev_image, cv_image)
        # self.three_image = self.prev_image
        # self.prev_image = cv_image

        # rospy.loginfo(f"Movement detected = {true}")
        cv2.waitKey(1)


    def three_way_intersection(self, cv_image):
        """
        @brief process image for truck intersection, display results
        """
        #Convert frame to binary
        height, width, _ = cv_image.shape
        cv_image = cv_image[height // 2:,:,:]
        threshold = 200
        blur_frame = cv2.GaussianBlur(cv_image, (5, 5), 0)
        gray_frame = blur_frame[:,:,1]
        _, img_bin = cv2.threshold(gray_frame, threshold, 255, cv2.THRESH_BINARY)
        # cv2.imshow("Bin Feed 1", img_bin)
        # cv2.imshow("No Wall Feed 1", frame_no_wall)
        # Get image dimensions

        cv2.imshow("Bin", img_bin)

    
    def main_intersection(self, cv_image):
        """
        @brief alternative process image for truck intersection, display results

        @note this serves the same purpose as three_way_intersection 
        """
        #Convert frame to binary
        height, width, _ = cv_image.shape
        line_image = cv_image[height // 4: 3 * height // 4, :]
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
        cv2.imshow("Green", line_image)

        # Apply thresholding to detect white areas
        _, binary = cv2.threshold(line_image, 100, 255, cv2.THRESH_BINARY)  # Adjust 200 if needed

        binary = binary[height_after // 3: 2 * height_after //3, width // 3: 2 * width // 3]

        cv2.imshow("Bin Feed 1", binary)
        # cv2.imshow("No Wall Feed 1", frame_no_wall)
        # Get image dimensions
        # height, width = img_bin.shape
        # Turn the top half white
        # img_bin[:height // 2, :] = [255]

        # cv2.imshow("Bin", img_bin)

    def main_road(self, cv_image):
        """
        @brief process image for the main road for line following, display results
        """
        #Convert frame to binary
        blur_frame = cv2.GaussianBlur(cv_image, (5, 5), 0)
        # gray_frame = cv2.cvtColor(blur_frame, cv2.COLOR_BGR2GRAY)
        gray_frame = blur_frame[:,:,1]
        frame_no_wall = np.where((gray_frame < self.wall_threshold), 255, gray_frame)
        _, img_bin = cv2.threshold(frame_no_wall, self.threshold, 255, cv2.THRESH_BINARY)
        # cv2.imshow("Bin Feed 1", img_bin)
        # cv2.imshow("No Wall Feed 1", frame_no_wall)
        # Get image dimensions
        height, width = img_bin.shape
        # Turn the top half white
        img_bin[:height // 2, :] = [255]

        cv2.imshow("Bin", img_bin)

    def grass_road2(self, cv_image):
        """
        @brief process image for the grass road, v2, display results
        """
        height, _, _ = cv_image.shape
        cv_image = cv_image[height // 2:,:,:]

        cv_image[:,:,1] = cv_image[:,:,0]
        cv_image[:,:,2] = cv_image[:,:,0]

        value = 40
        hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        # # Apply Contrast Limited Adaptive Histogram Equalization (CLAHE)
        # clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        # v = clahe.apply(v)
        

        # hsv = cv2.merge((h, s, v))
        hsv[:,:,2] = cv2.add(hsv[:,:,2], value)
        img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        
        blur_frame = cv2.GaussianBlur(img, (5, 5), 0)

        # Define structuring element
        erode_kernel = np.ones((7,7), np.uint8)

        # Perform erosion
        erosion = cv2.erode(blur_frame, erode_kernel, iterations = 2)

        dilate_kernel = np.ones((5,5),np.uint8)

        dilated_img = cv2.dilate(erosion, dilate_kernel, iterations=1)

        blur_post_erode = cv2.GaussianBlur(dilated_img,(5,5),0)

        hsv = cv2.cvtColor(blur_post_erode,cv2.COLOR_BGR2HSV)

        # Define HSV range for grass
        lower = np.array([0, 0, 150])  # Lower bound 
        upper = np.array([255, 140, 255])  # Upper bound 

        # Create mask
        grass_mask = cv2.inRange(hsv, lower, upper)

         # Find contours
        contours, _ = cv2.findContours(grass_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Create a fully white image
        final_img = np.ones_like(grass_mask) * 255

        min_contour_area = 250  # Adjust this value based on your setup
        for contour in contours:
            if cv2.contourArea(contour) > min_contour_area:
                cv2.drawContours(final_img, [contour], -1, (0), thickness=cv2.FILLED)  # Draw black lines
        img_bin = cv2.bitwise_not(final_img)

        # return img_bin

        # Debugging: Show the results
        # cv2.imshow("Draw", line_mask)
        cv2.imshow("Final", final_img)
        cv2.imshow("HSV Increase", blur_post_erode)
        cv2.imshow("HSV Mask", grass_mask)
        cv2.imshow("Erosion", erosion)


    def grass_road(self, cv_image):
        """
        @brief process image for grass road, v1 and final version, display results
        """
        #Convert frame to binary
        height, _, _ = cv_image.shape
        cv_image = cv_image[height // 2: 4 * height // 5,:,:]
        #gray_frame = cv2.cvtColor(blur_frame, cv2.COLOR_BGR2GRAY)

        # value = 40
        # hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        # h, s, v = cv2.split(hsv)
        # # Apply Contrast Limited Adaptive Histogram Equalization (CLAHE)
        # clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        # v = clahe.apply(v)
        

        # hsv = cv2.merge((h, s, v))
        # hsv[:,:,2] = cv2.add(hsv[:,:,2], value)
        # img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

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

        dilate_kernel = np.ones((7,7),np.uint8)

        dilated_img = cv2.dilate(erosion, dilate_kernel, iterations=1)

        # blur_frame = cv2.GaussianBlur(img, (5, 5), 0)

         # Find contours
        contours, _ = cv2.findContours(dilated_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Create a fully white image
        final_img = np.ones_like(grass_mask) * 255

        min_contour_area = 150  # Adjust this value based on your setup
        for contour in contours:
            if cv2.contourArea(contour) > min_contour_area:
                cv2.drawContours(final_img, [contour], -1, (0), thickness=cv2.FILLED)  # Draw black lines
        img_bin = cv2.bitwise_not(final_img)
        dilate_kernel = np.ones((7,7),np.uint8)

        final_img = cv2.dilate(final_img, dilate_kernel, iterations=1)
        # return img_bin

        # Debugging: Show the results
        # cv2.imshow("Draw", line_mask)
        cv2.imshow("Final", final_img)
        # cv2.imshow("HSV Increase", img)
        cv2.imshow("HSV Mask", grass_mask)
        cv2.imshow("Erosion", erosion)
        

    def test_find_blue (self, cv_image, left):
        """
        @brief process image for finding blue clueboards, display results
        """
        height, width, _ = cv_image.shape

        if left:
            cv_image = cv_image[7 * height // 12:2 * height // 3,: width // 8,:]
        else:
            cv_image = cv_image[7 * height // 12:2 * height // 3,7 * width // 8:,:]

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define HSV range for blue
        lower_blue = np.array([90, 90, 90])  # Lower bound for blue
        upper_blue = np.array([130, 255, 255])  # Upper bound for blue

        # Create mask
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Convert all previously white pixels (255) to black (0)
        processed_image = cv2.bitwise_and(cv_image, cv_image, mask=blue_mask)

        line_image = cv2.cvtColor(processed_image, cv2.COLOR_BGR2GRAY)

        _, img_bin = cv2.threshold(line_image, 10, 255, cv2.THRESH_BINARY)

        cv2.imshow("Blue",img_bin)
        cv2.imshow("pro", processed_image)
        cv2.waitKey(1)

    def test_red_and_fuchsia (self,cv_image):
        """
        @brief process image for finding stopping lines in red and fuchsia, display results
        """
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

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
        processed_image = cv2.bitwise_and(cv_image, cv_image, mask=red_mask)

        # Extract only the red channel
        red_channel = processed_image[:, :, 2]  # Extract the R channel from BGR

        # Show the result
        cv2.imshow("Red Line", red_channel)

        # Define HSV range for fuchsia/magenta
        lower_fuchsia = np.array([140, 100, 100])  # Lower bound
        upper_fuchsia = np.array([165, 255, 255])  # Upper bound

        # Create a mask
        fuchsia_mask = cv2.inRange(hsv, lower_fuchsia, upper_fuchsia)

        # Remove any white pixels (255) from previous processing
        processed_image = cv2.bitwise_and(cv_image, cv_image, mask=fuchsia_mask)

        # Extract red and blue channels
        red_channel = processed_image[:, :, 2]
        blue_channel = processed_image[:, :, 0]

        # Combine red and blue channels
        fuchsia_only = cv2.addWeighted(red_channel, 0.5, blue_channel, 0.5, 0)

        height, width = fuchsia_only.shape
        fuchsia_only = fuchsia_only[3* height // 4:, :]

        # Show the result
        cv2.imshow("Fuchsia Extracted", fuchsia_only)

        cv2.waitKey(1)

    def run(self):
        """
        @brief run CameraTesting and start camera
        """
        rospy.loginfo("Starting Camera :)")
        self.start_timer()  # Start timer

        while not rospy.is_shutdown():
            elapsed_time = rospy.Time.now().to_sec() - self.start_time

            rospy.sleep(0.1) #This was recommended by ChatGPT to not overwhelm the CPU

        rospy.spin()

        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        cam = CameraTesting()
        cam.run()
    except rospy.ROSInterruptException:
        pass
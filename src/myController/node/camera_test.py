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

        rospy.sleep(2)  # Ensure publishers are ready

        rospy.on_shutdown(self.stop_timer)  # Ensure the timer stops when script ends

    def start_timer(self):
        msg = f"{TEAM_NAME},{PASSWORD},0,NA"
        rospy.loginfo(f"Starting timer: {msg}")
        self.timer_pub.publish(msg)
        self.start_time = rospy.Time.now().to_sec()
        self.timer_started = True


    def stop_timer(self):
        if self.timer_started == True:
            msg = f"{TEAM_NAME},{PASSWORD},-1,NA"
            rospy.loginfo(f"Stopping timer: {msg}")
            self.timer_pub.publish(msg)
            self.timer_ended = True

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")
        cv2.imshow("Input", cv_image)

        # self.test_find_blue(cv_image, True)

        self.grass_road(cv_image)

        # true = self.road.detect_movement(self.prev_image, cv_image)
        self.prev_image = cv_image

        # rospy.loginfo(f"Movement detected = {true}")
        cv2.waitKey(1)


    def three_way_intersection(self, cv_image):
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

        
    def main_road(self, cv_image):
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

    def grass_road(self, cv_image):
        #Convert frame to binary
        height, _, _ = cv_image.shape
        cv_image = cv_image[height // 2:,:,:]
        blur_frame = cv2.GaussianBlur(cv_image, (5, 5), 0)
        #gray_frame = cv2.cvtColor(blur_frame, cv2.COLOR_BGR2GRAY)

        hsv = cv2.cvtColor(blur_frame, cv2.COLOR_BGR2HSV)

        # Extract the Value channel (brightness)
        _, _, v = cv2.split(hsv)

        # Define a threshold to extract bright regions (white lines)
        _, mask = cv2.threshold(v, 180, 190, cv2.THRESH_BINARY)  # Adjust 200 if needed

        # Remove small noise using morphological operations
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        # Invert mask so white background with black lines
        final_img = cv2.bitwise_not(mask)

        # Debugging: Show the results
        cv2.imshow("HSV Mask", mask)
        cv2.imshow("Final Image", final_img)
        

    def test_find_blue (self, cv_image, left):
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
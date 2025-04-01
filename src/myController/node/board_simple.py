#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class BoardDetector:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('board_detector', anonymous=True)
        self.bridge = CvBridge()

        # Subscribe to the camera feed topic
        self.image_sub = rospy.Subscriber('/B1/rrbot/camera1/image_raw', Image, self.image_callback)

        # Define the lower and upper bounds for detecting blue in HSV space
        self.lower_blue = np.array([90, 90, 50])  # Lower bound of blue
        self.upper_blue = np.array([130, 255, 255])  # Upper bound of blue

        self.board_detected = False  # Store whether a board was detected

    def board_in_frame(self, image):
        if image is None:
            print("Error: No image")
            return False

        # Convert the image to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create a mask for blue colors
        blue_mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)

        # Eliminate the top 40% of the image (if needed)
        height = blue_mask.shape[0]
        cutoff = int(0.4 * height)
        blue_mask[0:cutoff, :] = 0  # Mask off the top 40%

        # Find contours in the blue mask
        contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Check for contours with sufficient area (likely the board)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 5000:  # Adjust threshold for board size
                return True  # Board detected

        return False  # No board detected

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print("Error converting image:", e)
            return

        # Pass the image to the board detection function
        self.board_detected = self.board_in_frame(cv_image)

    def get_board_detection_status(self):
        return self.board_detected

    def run(self):
        # Keep the node running and process messages
        rate = rospy.Rate(1)  # 1 Hz, print every second
        while not rospy.is_shutdown():
            print("Board detected:", self.get_board_detection_status())
            rate.sleep()  # Sleep for the specified rate (1 second)

if __name__ == '__main__':
    detector = BoardDetector()  # Initialize the board detector class
    detector.run()  # Run the ROS spin loop

    # If you want to check the detection status, you can call:
    board_status = detector.get_board_detection_status()
    print("Board detected:", board_status)

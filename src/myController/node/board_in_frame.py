#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class boardDetector:
    def __init__(self):
        rospy.init_node('board_detector', anonymous=True)
        print("ROS Node initialized.")  # Debugging statement to verify initialization
        self.bridge = CvBridge()

        # Subscribe to the robot's camera image topic
        self.image_sub = rospy.Subscriber('/B1/rrbot/camera1/image_raw', Image, self.image_callback)
        print("Subscriber created.")  # Debugging statement

        self.lower_blue = np.array([90, 90, 50])  # Lower bound of blue
        self.upper_blue = np.array([130, 255, 255])  # Upper bound of blue

        # Create a named OpenCV window
        cv2.namedWindow("Board Detection", cv2.WINDOW_NORMAL)

    def board_in_frame(self, image):
        if image is None:
            print(f"Error: no image")
            return False, image

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

    def image_callback(self, msg):
        print("Image received.")
        try:
            # Convert the ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print("Error converting image:", e)
            return
        
        # Pass the frame to the board_in_frame function
        found, image_with_bbox = self.board_in_frame(cv_image)
    
        # Display the result
        if found:
            print("Board detected!")
        else:
            print("No board detected.")
    
        # Show the image with the bounding box (if detected)
        cv2.imshow("Board Detection", image_with_bbox)
        cv2.waitKey(1)  # Important to update the image window
        print("Callback finished.")

    def run(self):
        print("Node is running...")  # Debugging statement to verify that the spin loop is running
        try:
            rospy.spin()  # Keep the node alive and processing messages
        except rospy.ROSInterruptException:
            print("ROS Interrupt exception occurred.")  # If ROS is interrupted
        except Exception as e:
            print(f"Unexpected error occurred: {e}")

if __name__ == '__main__':
    follower = boardDetector()  # Create an instance of the boardDetector class
    follower.run()  # Run the node

    # Create a static test image
    #bridge = CvBridge()
    #image = cv2.imread('/home/fizzer/ros_ws/src/imgRecog/sampleImages/image1.png')  # Replace with your own image path
    #msg = bridge.cv2_to_imgmsg(image, "bgr8")

    # Manually call the image callback
    #board_detector = boardDetector()
    #while(True):
    #    board_detector.image_callback(msg)


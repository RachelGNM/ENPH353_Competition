#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

TEAM_NAME = "Smithies"
PASSWORD = "Volcan"

class LineFollower:
    def __init__(self):
        rospy.init_node('time_trials', anonymous=True)
        
        # Publishers
        self.timer_pub = rospy.Publisher('/score_tracker', String, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.sleep(2)  # Ensure publishers are ready

        self.start_timer()  # Start timer when launching
        self.endpoint = 15
        self.start_time = rospy.Time.now().to_sec()
        rospy.on_shutdown(self.stop_timer)  # Ensure the timer stops when script ends

    def start_timer(self):
        msg = f"{TEAM_NAME},{PASSWORD},0,NA"
        rospy.loginfo(f"Starting timer: {msg}")
        self.timer_pub.publish(msg)

    def stop_timer(self):
        msg = f"{TEAM_NAME},{PASSWORD},-1,NA"
        rospy.loginfo(f"Stopping timer: {msg}")
        self.timer_pub.publish(msg)

        #Stop the robot
        self.move.linear.x = 0
        self.move.angular.z = 0
        self.cmd_vel_pub.publish(self.move)

    
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")

        #Convert frame to binary
        blur_frame = cv2.GaussianBlur(cv_image, (5, 5), 0)
        gray_frame = cv2.cvtColor(blur_frame, cv2.COLOR_BGR2GRAY)
        _, img_bin = cv2.threshold(gray_frame, threshold, 255, cv2.THRESH_BINARY)
        # Get image dimensions
        height, width = img_bin.shape
        # Turn the top half white
        img_bin[:2 * height // 3, :] = [255]


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
                self.move.angular.z = turn
                if abs(turn) < 0.5:
                    self.move.linear.x = 0.2
                elif abs(turn) < 0.7:
                    self.move.linear.x = 0.1
                else:
                    self.move.angular.z = 0.3 * turn
                    self.move.linear.x = 0.08
                self.cmd_vel_pub.publish(move)

            else:
                self.move.angular.z = 1.0
                self.move.linear.x = 0
        else:
            self.move.angular.z = 1.0
            self.move.linear.x = 0

        
        cv2.imshow("Bin Feed", img_bin)
        cv2.waitKey(1)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            elapsed_time = rospy.Time.now().to_sec() - self.start_time

            if elapsed_time >= self.endpoint:
                rospy.loginfo(f"Time limit reached ({self.endpoint} seconds). Stopping robot.")
                self.stop_timer
                break #exit loop after time limit
            
            rate.sleep()
        cv2.destroyAllWindows


if __name__ == '__main__':
    try:
        follower = LineFollower()
        follower.follow_line()
    except rospy.ROSInterruptException:
        pass

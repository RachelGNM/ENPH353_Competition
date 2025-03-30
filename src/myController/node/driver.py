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

class Driver:
    def __init__(self):
        rospy.init_node('driver', anonymous=True)

        self.road_reader = RoadProcessing

        self.timer_started = False
        self.timer_ended = False
        self.start_time = None
        self.endpoint = 240

        self.zone = 0
        # self.clues_read[] = [0] * 8
        self.clue = 0
        self.find_clueboard = False
        
        # Publishers
        self.timer_pub = rospy.Publisher('/score_tracker', String, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)

        self.image_sub = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.image_callback)

        self.bridge=CvBridge()

        self.threshold = 100
        self.Kp = 0.6
        self.Kd = 0.2
        self.move = Twist()
        self.last_error = 0


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

        #Stop the robot
        self.move.linear.x = 0
        self.move.angular.z = 0
        self.cmd_vel_pub.publish(self.move)

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")
        
        while True:
            img_bin = self.road_reader.road_binarize(cv_image, self.zone)

            if self.road_reader.stopping_point(cv_image, self.zone):
                break
            line_follow(,img_bin)

        #TODO: use function here to check for a clueboard, if it exists, read it and increment self.clue (assuming we're going in order)
        #author: Alfred
        #param: input image, outputs find_clueboard = true

    def line_follow(self, img_bin):
        # Find contours
        contours, _ = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Process only if at least one contour is found

        if self.timer_ended == False:
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

                    turn = self.Kp * error + self.Kd * (error - self.last_error) / 2
                    self.last_error = error

                    self.move.angular.z = turn
                    if abs(turn) < 0.5:
                        self.move.linear.x = 1
                    elif abs(turn) < 0.7:
                        self.move.linear.x = 0.5
                    else:
                        self.move.angular.z = turn
                        self.move.linear.x = 0.08
                    self.cmd_vel_pub.publish(self.move)

                else:
                    self.move.linear.x = -0.2
            else:
                self.move.linear.x = -0.2
        else:
            #Stop the robot
            self.move.linear.x = 0
            self.move.angular.z = 0
            self.cmd_vel_pub.publish(self.move)
        
    def run(self):
        rospy.loginfo("Starting Comp :)")
        self.start_timer()  # Start timer


        while not rospy.is_shutdown():
            elapsed_time = rospy.Time.now().to_sec() - self.start_time

            if elapsed_time >= self.endpoint:
                rospy.loginfo(f"Time limit reached ({self.endpoint} seconds). Stopping robot.")
                self.stop_timer()
                rospy.loginfo("Stopping Comp :(")
                break  # Exit loop after time limit

            rospy.sleep(0.1) #This was recommended by ChatGPT to not overwhelm the CPU

        rospy.spin()

        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        driver = Driver()
        driver.run()
    except rospy.ROSInterruptException:
        pass
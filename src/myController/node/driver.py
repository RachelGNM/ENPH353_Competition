#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

TEAM_NAME = "Smithies"
PASSWORD = "Volcan"

class Driver:
    def __init__(self):
        rospy.init_node('driver', anonymous=True)

        self.timer_started = False
        self.timer_ended = False
        self.start_time = None
        self.endpoint = 240

        self.zone = 0
        self.clue = 0
        
        # Publishers
        self.timer_pub = rospy.Publisher('/score_tracker', String, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)

        self.image_sub = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.image_callback)

        self.bridge=CvBridge()

        self.threshold = 100
        self.Kp = 0.5
        self.move = Twist()

        rospy.sleep(2)  # Ensure publishers are ready

        rospy.on_shutdown(self.stop_timer)  # Ensure the timer stops when script ends

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")
        
        #TODO: make a code that decides what it's looking for
        # @param self.zone (zone 0 is the beginning until the pedestrian, zone 1 is the pedestrian, zone 2 is the car, zone 3 is the new biome)
        # @param: self.clue references which clue we're at

        if self.zone == 0 AND self.clue == 0:
            if check_red() == false:
                
                


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
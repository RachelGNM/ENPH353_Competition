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

        #Subscribe to camera topic
        self.image_sub = ropsy.Subscriber("/rrbot/camera1/image_raw", Image, self.image_callback)

        #Initialize CVBridge
        self.bridge = CvBridge()

        #Line following variables
        self.threshold = 120 #threshold for binary
        #Proportional control constants
        self.Kp = 0.5
        self.move = Twist()

        rospy.sleep(2)  # Ensure publishers are ready

        self.start_timer()  # Start timer when launching
        rospy.on_shutdown(self.stop_timer)  # Ensure the timer stops when script ends

    def start_timer(self):
        msg = f"{TEAM_NAME},{PASSWORD},0,NA"
        rospy.loginfo(f"Starting timer: {msg}")
        self.timer_pub.publish(msg)

    def stop_timer(self):
        msg = f"{TEAM_NAME},{PASSWORD},-1,NA"
        rospy.loginfo(f"Stopping timer: {msg}")
        self.timer_pub.publish(msg)

    def follow_line(self):
        rate = rospy.Rate(10)  # Run at 10 Hz

        while not rospy.is_shutdown():
            # Your line-following logic goes here
            move_cmd = Twist()
            move_cmd.linear.x = 0.2  # Example forward speed
            self.cmd_vel_pub.publish(move_cmd)

            # Add logic to detect the finish line and break loop
            rate.sleep()

        self.stop_timer()  # Stop the timer when the task is done

if __name__ == '__main__':
    try:
        follower = LineFollower()
        follower.follow_line()
    except rospy.ROSInterruptException:
        pass

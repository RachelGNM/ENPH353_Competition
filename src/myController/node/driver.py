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

        self.road_reader = RoadProcessing()

        self.timer_started = False
        self.timer_ended = False
        self.start_time = None
        self.endpoint = 120

        #this is to map where the robot is on the map
        self.zone = 0
        self.clue = 0

        #for clue reading logic
        self.clues_seen = 0
        self.ready_to_read = False
        self.clue_isRead = False
        #self.find_clueboard = False
        
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

        #These are for finding movement
        self.previous_image = np.zeros((800,800,3), dtype=np.uint8)
        self.prev_waiting = False
        self.obstacle = False
        #self.waiting = False this was replaced by self.obstacle cuz I need less stuff with the same names


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

        #Look for all scenarios which require anything other than generic line following
        self.zone, stopping_line = self.road_reader.stopping_point(cv_image, self.zone)
        truck = (self.zone == 3)
        self.clues_seen, clue_spotted = self.road_reader.detect_sign(cv_image,self.clue)
        new_clue = (self.clues_seen != self.clue)
        stop = stopping_line or (self.zone == 6) or truck or new_clue

        if stop == False:
            img_bin = self.road_reader.road_binarize(cv_image, self.zone)
            self.prev_waiting = False
            self.obstacle = False
            self.line_follow(img_bin)
        else:
            rospy.loginfo("Stop started")
            #make sure it actually stops
            self.move.linear.x = 0
            self.move.angular.z = 0

            #This is checking for clueboards first ahead of zone switches
            if new_clue:
                rospy.loginfo("Looking for clueboard")
                direction = 0
                if (self.clue % 2) == 0:
                    rospy.loginfo("Turning left for clueboard")
                    direction = -1
                else:
                    rospy.loginfo("Turning right for clueboard")
                    #Same as above, opposite direction
                    direction = 1
                findclue = self.wait_for_clue(cv_image,direction)
                if findclue:
                    self.clue = self.clues_seen
                    if self.clue == 3:
                        self.zone = 3

            #make actions dependent on zone. I need to make a map of these zones for myself in my logbook
            if self.zone == 1:
                rospy.loginfo("Stop at zone 1")
                movement = self.wait_for_movement(cv_image, 1, 0, 1)
                if movement == True:
                    self.zone = 2
            elif self.zone == 2:
                #TODO: Wait for truck then turn left, line follow, and turn left at intersection again
                #Must somehow stay left at the end. Also increment zone after getting past the loop
                self.move.linear.x = 0
            elif self.zone == 4: #this is reaching the new biome
                #Just move forward until stop == false
                self.move.linear.x = 0.8
            elif self.zone == 6:
                #TODO: Wait for Yoda to pass then hard-code path through grassland
                self.move.linear.x = 0
            elif self.zone == 7: #This is right after passing the Yoda land, entering the tunnel
                #TODO: Yoda-land should be completed with car facing the correct way to line-follow
                self.move.linear.x = 0.8
        self.cmd_vel_pub.publish(self.move)

        cv2.imshow("camera feed", cv_image)

        previous_image = cv_image
        #TODO: use function here to check for a clueboard, if it exists, read it and increment self.clue (assuming we're going in order)
        #author: Alfred
        #param: input image, outputs find_clueboard = true

    def line_follow(self, img_bin):
        # Find contours
        contours, _ = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Process only if at least one contour is found
        height, width = img_bin.shape

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

    def wait_for_movement(self, image, forward_movement, turn, delay):
        #Wait for thing to cross, then zoom through
        if self.prev_waiting:
            rospy.loginfo("Previous waiting was activated")
            if self.obstacle:
                #if there is movement found and we're waiting, then run forward
                if self.road_reader.detect_movement(self.previous_image, image):
                    self.move.linear.x = forward_movement
                    self.move.angular.z = turn
                    #put a delay here so it actually goes through
                    self.cmd_vel_pub.publish(self.move)
                    return True
            else:
                rospy.loginfo("Waiting to detect movement")
                #if there has been no previous movement found, wait till movement is found
                if self.road_reader.detect_movement(self.previous_image, image):
                    self.obstacle = True
                else:
                    self.move.linear.x = 0
                    self.move.angular.z = 0
        else:
            rospy.loginfo("Robot initial stop after stopping point detected")
            #The robot stops moving after initial stop
            self.move.linear.x = 0
            self.move.angular.z = 0
            self.prev_waiting = True
        self.cmd_vel_pub.publish(self.move)
        return False

    def wait_for_clue(self, image, direction):
        #Wait for thing to cross, then zoom through
        #TODO: insert Alfred's recognize clueboard function here
        if self.ready_to_read: #TODO: this should depend on clueboard finding function
            self.ready_to_read = True

        if self.ready_to_read:
            rospy.loginfo("Robot has full view of clueboard")
            if self.clue_isRead:
                #if the clueboard has been read, turn back towards the road
                self.move.angular.z = direction 
                #TODO: insert delay or other way to get back to the road
                self.cmd_vel_pub.publish(self.move)
                return True
            else:
                rospy.loginfo("Waiting to read clueboard")
                #if there has been no clue read, must read the clueboard
                if True: #TODO: insert Alfred's function here to read a clueboard
                    self.clue_isRead = True
        else:
            rospy.loginfo("Robot found clueboard, rotating to find clueboard")
            #The robot stops moving after initial stop
            self.move.linear.x = 0
            self.move.angular.z = direction
        self.cmd_vel_pub.publish(self.move)
        return False

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

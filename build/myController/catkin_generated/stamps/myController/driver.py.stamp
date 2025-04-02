#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

import sys
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__)))

from road_processing import RoadProcessing
from motion_detector import MotionDetector
from side_camera import SideCam
# from clue_reader import clueReader
# from board_in_frame import boardDetector

TEAM_NAME = "Smithies"
PASSWORD = "Volcan"

class Driver:
    def __init__(self):
        rospy.init_node('driver', anonymous=True)

        self.road_reader = RoadProcessing()
        self.motion_detector = MotionDetector()
        self.sidecam = SideCam()
        # self.clue_reader = clueReader()

        # self.timer_started = False
        self.timer_ended = False
        self.start_time = None
        self.endpoint = 60

        #this is to map where the robot is on the map
        self.zone = 0
        self.clue = 0
        self.time_zone = 0

        #for clue reading logic
        self.clues_seen = 0
        self.ready_to_read = False
        self.prev_read = False
        self.clue_isRead = False
        self.prev_clue = ""
        self.time_clue = None
        # self.found_clueboard = None
        
        # Publishers
        self.timer_pub = rospy.Publisher('/score_tracker', String, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)

        self.image_sub = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.image_callback)

        self.bridge=CvBridge()

        #Movement stuff
        self.threshold = 100
        self.Kp = 0.6
        self.Kd = 0.3
        self.move = Twist()
        self.last_error = 0

        #Pedestrian movement
        # self.movement_start_time = None  # Time when the movement started

        #These are for finding movement
        self.previous_image = np.zeros((800,800,3), dtype=np.uint8)
        self.prev_waiting = False
        self.obstacle = False
        self.obstacle_passed = False
        self.increment = 0
        self.found_left = False
        #self.waiting = False this was replaced by self.obstacle cuz I need less stuff with the same names

        #To average out movement so that there is momentum when in the grassland
        self.prev_len = 0
        self.prev_ang = 0

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
        # rospy.loginfo(f"Truck: {truck}")
        self.clues_seen, clue_spotted = self.road_reader.detect_sign(cv_image,self.clue)
        # new_clue = (self.clues_seen != self.clue)
        # current_time = rospy.Time.now().to_sec()
        # rospy.loginfo(f"Current time set to: {current_time}")
        # current_delay = current_time - self.time_zone
        # rospy.loginfo(f"Current delay set to: {current_delay}")
        # if self.zone == 2:
        #     stopping_line = self.road_reader.find_intersection(cv_image)
        truck = (self.zone == 3)
        stop = stopping_line or (self.zone == 6) or truck

        if stop == False:
            # if clue_spotted:
            #     left = False
            #     if self.clue % 2 == 0:
            #         left = True
            #     self.ready_to_read, _ = self.sidecam.process_image(left)
            #     if not self.prev_read and self.ready_to_read:
            #         self.clue += 1
            #         rospy.loginfo(f"New clue: {self.clue}!")
            #     self.prev_read = self.ready_to_read 
            img_bin = self.road_reader.road_binarize(cv_image, self.zone)
            self.prev_waiting = False
            self.obstacle = False
            self.increment = 0
            if self.zone < 5:
                self.line_follow(img_bin)
            else:
                self.line_follow_grass(img_bin)
        else:
            # rospy.loginfo("Stop started")
            rospy.loginfo(f"Zone: {self.zone}")
            # make sure it actually stops
            self.move.linear.x = 0
            self.move.angular.z = 0

            #make actions dependent on zone. I need to make a map of these zones for myself in my logbook
            if self.zone == 1:
                # rospy.loginfo("Stop at zone 1")
                self.obstacle_passed = self.wait_for_movement(cv_image, 1, 0, 2.5)
                if self.obstacle_passed:
                    rospy.loginfo("Zone 1 Complete!")
                    self.zone = 2
                    self.obstacle = False
                    self.prev_waiting = False
                    self.obstacle_passed = False
                    # self.time_zone = rospy.Time.now().to_sec()
                    # rospy.loginfo(f"Time zone set to: {self.time_zone}")
            elif self.zone == 2 and self.clue == 3:
                rospy.loginfo("Moving onto zone 3!")
                self.zone = 3
            elif self.zone == 3:
                #Wait for truck then turn left, line follow, and turn left at intersection again
                #Must somehow stay left at the end. Also increment zone after getting past the loop
                # rospy.loginfo("Stop at zone 3")
                if not self.obstacle_passed:
                    self.obstacle_passed = self.wait_for_movement(cv_image, 1, 1.5, 2)
                else:
                    img_bin = self.road_reader.road_binarize(cv_image, 3)
                    img_bin1 = self.road_reader.road_binarize(cv_image, 1)
                    height, width = img_bin.shape
                    # img_bin = img_bin[:,:width // 2]
                    # cv2.imshow("Left", img_bin)
                    # Turn the top section white for line following
                    if self.increment < 50:
                        self.line_follow(img_bin1)
                        self.increment += 1
                    elif self.road_reader.detect_left_turn(img_bin):
                        if self.found_left:
                            self.move.linear.x = 1
                            self.move.angular.z = 1.5
                            self.cmd_vel_pub.publish(self.move)
                            time.sleep(2)
                            self.move.linear.x = 0
                            self.move.angular.z = 0
                            self.cmd_vel_pub.publish(self.move)
                            self.zone = 4
                        else:
                            self.found_left = True
                    else:
                        # height, width = img_bin1.shape
                        img_bin = img_bin1[:,width // 2:]
                        # cv2.imshow("Left", img_bin)
                        self.line_follow(img_bin1)
                        self.found_left = False
            elif self.zone == 4: #this is reaching the new biome
                #Just move forward until stop == false
                self.move.linear.x = 0.8
                self.cmd_vel_pub.publish(self.move)
                time.sleep(1)
                self.zone == 5
            elif self.zone == 5:
                self.move.linear.x = 0
                self.move.angular.z = 0
                self.cmd_vel_pub.publish(self.move)
                time.sleep(0.5)
                self.zone = 6
            elif self.zone == 6:
                #TODO: Wait for Yoda to pass then hard-code path through grassland
                self.move.linear.x = 0
            elif self.zone == 7: #This is right after passing the Yoda land, entering the tunnel
                #TODO: Yoda-land should be completed with car facing the correct way to line-follow
                self.move.linear.x = 0.8
        self.cmd_vel_pub.publish(self.move)

        # cv2.imshow("camera feed", cv_image)

        previous_image = cv_image
        #TODO: use function here to check for a clueboard, if it exists, read it and increment self.clue (assuming we're going in order)
        #author: Alfred
        #param: input image, outputs find_clueboard = true

    def pause(self):
        delay_stop = 1
        go = False
        if self.clue == 0:
            delay_go = 2.5
            go = True
        else:
            delay_go = 0.5
            go = True
        self.move.linear.x = 0
        self.move.angular.z = 0
        self.cmd_vel_pub.publish(self.move)
        time.sleep(delay_stop)
        if go:
            self.move.linear.x = 1
            self.cmd_vel_pub.publish(self.move)
            time.sleep(delay_go)

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

    def line_follow_grass(self, img_bin):
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
                    #Centroid: ({cx}, {cy})
                    error = cx - width / 2

                    turn = self.Kp * error + self.Kd * (error - self.last_error) / 2
                    self.last_error = error

                    self.move.angular.z = turn / 2
                    if abs(turn) < 0.5:
                        self.move.linear.x = 0.5
                    elif abs(turn) < 0.7:
                        self.move.linear.x = 0.25
                    else:
                        self.move.angular.z = turn / 2
                        self.move.linear.x = 0.04
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
                if not self.motion_detector.detect_movement(image, self.zone):
                    time.sleep(1)
                    self.move.linear.x = forward_movement
                    self.move.angular.z = turn
                    self.cmd_vel_pub.publish(self.move)
                    rospy.loginfo("Movement started!")
                    time.sleep(delay)
                    # Stop the robot after 1 second
                    self.move.linear.x = 0
                    self.move.angular.z = 0
                    self.cmd_vel_pub.publish(self.move)
                    rospy.loginfo("Movement duration completed, stopping robot.")
                    return True
            else:
                # rospy.loginfo("Waiting to detect movement")
                #if there has been no previous movement found, wait till movement is found
                if self.motion_detector.detect_movement(image, self.zone):
                    rospy.loginfo("Movement detected")
                    self.obstacle = True
                else:
                    rospy.loginfo("No movement, waiting for obstacle")
                    self.move.linear.x = 0
                    self.move.angular.z = 0
        else:
            rospy.loginfo("Robot initial stop after stopping point detected")
            #The robot stops moving after initial stop
            self.move.linear.x = 0
            self.move.angular.z = 0
            self.prev_waiting = True
        self.cmd_vel_pub.publish(self.move)
        time.sleep(0.05)
        return False

    def look_for_clue(self, image, direction):
        #This is to turn towards the signs when we see them
        if self.ready_to_read:
            rospy.loginfo("Robot has full view of clueboard")
            if self.clue_isRead:
                #if the clueboard has been read, turn back towards the road
                self.move.angular.z = - direction 
                self.cmd_vel_pub.publish(self.move)
                time.sleep(0.5)
                self.clue_isRead = False
                return True
            else:
                rospy.loginfo("Waiting to read clueboard")
                self.move.linear.x = 0
                self.move.angular.z = 0
                self.cmd_vel_pub.publish(self.move)
                time.sleep(1)
                #if there has been no clue read, must read the clueboard
                if True: #TODO: insert Alfred's function here to read a clueboard
                    self.clue_isRead = True
        else:
            rospy.loginfo("Robot found clueboard, rotating to find clueboard")
            #The robot stops moving after initial stop
            self.move.linear.x = 0
            self.move.angular.z = direction
            #TODO: make ready_to_read == is full rectangle showing, should be Alfred's code
            self.ready_to_read = True
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
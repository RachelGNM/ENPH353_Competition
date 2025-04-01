#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import tensorflow as tf
from tensorflow.keras.models import load_model

"""
Node that looks for clues in the image feed and detects whether or not there is a board in the image (prints status message of detection).
If a board is detected, then the node reads what is on the board and prints what its read in the terminal.
"""


def load_image(path):
    return cv2.imread(path)

def binarize_image(image, threshold=100):
    """Convert to grayscale and binarize."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)
    return binary

def find_largest_contour(binary):
    """Find the largest contour, which should be the white text area box."""
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise ValueError("No contours found")
    return max(contours, key=cv2.contourArea)

def order_points(pts):
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1)

    rect[0] = pts[np.argmin(s)]       # Top-left
    rect[2] = pts[np.argmax(s)]       # Bottom-right
    rect[1] = pts[np.argmin(diff)]    # Top-right
    rect[3] = pts[np.argmax(diff)]    # Bottom-left

    return rect


def warp_perspective_to_rectangle(image, contour, output_size=(1200, 400)):
    epsilon = 0.02 * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)

    if len(approx) != 4:
        raise ValueError("Contour does not have 4 corners.")

    pts = approx.reshape(4, 2)
    ordered = order_points(pts)

    dst = np.array([
        [0, 0],
        [output_size[0] - 1, 0],
        [output_size[0] - 1, output_size[1] - 1],
        [0, output_size[1] - 1]
    ], dtype="float32")

    M = cv2.getPerspectiveTransform(ordered, dst)
    warped = cv2.warpPerspective(image, M, output_size)
    return warped


def extract_characters_by_contour(warped_image, y_crop_start=0, height=200, target_size=(108, 108)):
    """
    Crops HSV hue region and finds character contours, returning resized character images.
    Focuses on hues in the range 90–130.
    """
    # Crop region of interest
    region = warped_image[y_crop_start:y_crop_start + height, :]

    # Convert to HSV and threshold by hue range
    hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
    lower_hue = np.array([90, 90, 50])   # Adjust S/V thresholds if needed
    upper_hue = np.array([130, 255, 255])
    mask = cv2.inRange(hsv, lower_hue, upper_hue)

    # Morphological operations to clean up the mask
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
    binary = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Find contours in the binary mask
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    char_images = []
    boxes = []
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        if 10 < w < 160 and 20 < h < 200:
            char_img = binary[y:y+h, x:x+w]
            resized = cv2.resize(char_img, target_size)
            char_images.append(resized)
            boxes.append((x, y, w, h))

    # Sort characters left to right
    char_images = [char for _, char in sorted(zip(boxes, char_images), key=lambda b: b[0][0])]
    return char_images

class clueReader:
    def __init__(self):
        rospy.init_node('clue_reader', anonymous=True)

        self.last_detection_time = rospy.Time.now()
        self.cooldown_duration = rospy.Duration(0.5)  # 1 second between detections
        
        # Load the model
        model_path = "/home/fizzer/ros_ws/src/myController/models/clue__recog_cnn.h5"
        self.model = load_model(model_path)


        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, self.image_callback)
        self.score_pub = rospy.Publisher("/score_tracker", String, queue_size=10)

        rospy.loginfo("Clue Board Detector with CNN ready.")
        rospy.spin()

    def image_callback(self, msg):
        if rospy.Time.now() - self.last_detection_time < self.cooldown_duration:
            return
        self.last_detection_time = rospy.Time.now()

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"CV Bridge error: {e}")
            return

        binary = binarize_image(frame)

        try:
            contour = find_largest_contour(binary)
        except ValueError:
            rospy.loginfo("No clue board detected (no contours).")
            return

        try:
            # Visualize original frame with detected contour
            debug_img = frame.copy()
            cv2.drawContours(debug_img, [contour], -1, (0, 255, 0), 2)

            # Get perspective warp corners
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = box.astype(int)
            ordered_box = order_points(box)

            # Draw points and labels
            for i, pt in enumerate(ordered_box):
                pt = tuple(pt.astype(int))
                cv2.circle(debug_img, pt, 8, (0, 0, 255), -1)
                cv2.putText(debug_img, str(i), pt, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            cv2.imshow("Detected Contour + Points", debug_img)

            # Perform the warp
            warped = warp_perspective_to_rectangle(frame, contour)

            # Debug view
            cv2.imshow("Warped Board", warped)
            cv2.imshow("Binary Char Mask", binary)
            cv2.waitKey(1)

            # Show warped result
            cv2.imshow("Warped Board", warped)
            cv2.imshow("Binary Char Mask", binary)
            cv2.waitKey(1)

            # Extract characters
            char_images = extract_characters_by_contour(warped)

        except Exception as e:
            rospy.logwarn(f"Clue board detected but failed to extract characters: {e}")
            return

        if not char_images:
            rospy.loginfo("Clue board detected, but no characters found.")
            return

        rospy.loginfo("Clue board detected.")
        clue = ""
        for idx, char_img in enumerate(char_images):
            char_img = char_img.astype(np.float32) / 255.0
            char_img = cv2.cvtColor(char_img, cv2.COLOR_GRAY2RGB)
            char_img = np.expand_dims(char_img, axis=0)

            prediction = self.model.predict(char_img)[0]
            predicted_label = chr(np.argmax(prediction) + ord('A'))
            clue += predicted_label

        rospy.loginfo(f"Detected clue: {clue}")
        msg = f"TeamName,password,2,{clue}"  # Update with real values
        self.score_pub.publish(String(data=msg))

    
if __name__ == '__main__':
    clueReader()
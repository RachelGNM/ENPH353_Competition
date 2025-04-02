#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import csv
import os

import tensorflow as tf
from tensorflow.keras.models import load_model
from collections import defaultdict

"""
Node that looks for clues in the image feed and detects whether or not there is a board in the image (prints status message of detection).
If a board is detected, then the node reads what is on the board and prints what its read in the terminal.
"""

lower_blue = np.array([90, 90, 50])  # Lower bound of blue
upper_blue = np.array([130, 255, 255])  # Upper bound of blue

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

# Order points: top-left, top-right, bottom-right, bottom-left
def order_points(pts):
    """Helper function for perspective warp"""
    pts = pts[np.argsort(pts[:, 1])]  # sort by y
    top, bottom = pts[:2], pts[2:]
    top = top[np.argsort(top[:, 0])]
    bottom = bottom[np.argsort(bottom[:, 0])]
    return np.array([top[0], top[1], bottom[1], bottom[0]], dtype="float32")


def warp_perspective_to_rectangle(image, contour, output_size=(1200, 400)):
    """Correct skew and rotation using a perspective transform."""
    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect)
    box = box.astype(int)

    box = order_points(box)

    dst = np.array([
        [0, 0],
        [output_size[0] - 1, 0],
        [output_size[0] - 1, output_size[1] - 1],
        [0, output_size[1] - 1]
    ], dtype="float32")

    M = cv2.getPerspectiveTransform(box, dst)
    warped = cv2.warpPerspective(image, M, output_size)
    return warped

def get_board(image):
    """
    Finds and returns the largest clue board in the frame
    """

    if image is None:
        print(f"Error: no image")
        return False, image

    # Convert to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Create blue mask
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Eliminate top 40% of the image from the mask
    height = blue_mask.shape[0]
    cutoff = int(0.4 * height)
    blue_mask[0:cutoff, :] = 0  # Set top 40% to black (mask off)

    # Apply the modified mask
    result = cv2.bitwise_and(image, image, mask=blue_mask)

    # Find contours in the blue mask
    contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)

        # Crop ROI and upscale to 600x400
        roi = image[y:y+h, x:x+w]
        roi_upscaled = cv2.resize(roi, (600, 400), interpolation=cv2.INTER_CUBIC)

        rospy.loginfo("Clue board detected")
        return roi_upscaled
    else:
        rospy.logwarn("No clue board detected)")


def extract_characters_by_contour(warped_image, y_crop_start=200, height=200, target_size=(108, 108)):
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

        self.clue_location_lookup = {
            "S": 1,
            "C": 3,
            "P": 5,
            "W": 7,
        }


        self.last_detection_time = rospy.Time.now()
        self.cooldown_duration = rospy.Duration(0.2)  # 0.25 second between detections
        
        # Load the model
        model_path = "/home/fizzer/ros_ws/src/myController/models/clue__recog_cnn.h5"
        self.model = load_model(model_path)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/B1/rrbot/camera_left/image_left_raw", Image, self.image_callback)
        self.score_pub = rospy.Publisher("/score_tracker", String, queue_size=10)

        # Set up SQLite database
        db_path = os.path.expanduser("/home/fizzer/ros_ws/src/imgRecog/Lclue_database.csv")
        self.init_csv()


        rospy.loginfo("Clue Board Detector with CNN ready.")
        rospy.spin()

    def init_csv(self):
        self.csv_path = os.path.expanduser("/home/fizzer/ros_ws/src/imgRecog/Lclue_database.csv")
        self.clue_counts = defaultdict(int)

        # Start with a fresh file each run
        with open(self.csv_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Clue Type", "Clue Value", "Count"])  # Header

    def update_csv(self, clue_type, clue_value):
        key = (clue_type, clue_value)
        self.clue_counts[key] += 1

        # Write the entire file every update
        with open(self.csv_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Clue Type", "Clue Value", "Count"])
            for (ctype, cval), count in self.clue_counts.items():
                writer.writerow([ctype, cval, count])

        # Publish clue if seen 5 times
        if count == 2:
            rospy.loginfo(f"Publishing clue '{clue_value}' of type '{clue_type}' after 5 detections.")
            location = self.clue_location_lookup.get(clue_type[0], 0)  # default to 0 if unknown
            rospy.loginfo(f"TeamName,password,{location},{clue_value}")
            msg = f"TeamName,password,{location},{clue_value}"

            self.score_pub.publish(String(data=msg))


    def image_callback(self, msg):

        #Enforce cooldown to avoid overload
        if rospy.Time.now() - self.last_detection_time < self.cooldown_duration:
            return
        self.last_detection_time = rospy.Time.now()

        #Get image from camera feed and isolate the board
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"CV Bridge error: {e}")
            return
        
        cv2.imshow("left",frame)
        cv2.waitKey(1)
        
        board = get_board(frame)

        #Binarize board, then align board to account for perspective differences then extract characters
        binary = binarize_image(board)
        contour = find_largest_contour(binary)
        aligned = warp_perspective_to_rectangle(board, contour)
        characters = extract_characters_by_contour(aligned)
        clueChars = extract_characters_by_contour(aligned, y_crop_start=0)

        #Feed each recognised character into CNN to get the clue and type
        clue = ""
        for idx, char_img in enumerate(characters):
            char_img = char_img.astype(np.float32) / 255.0
            char_img = cv2.cvtColor(char_img, cv2.COLOR_GRAY2RGB)
            char_img = np.expand_dims(char_img, axis=0)

            prediction = self.model.predict(char_img)[0]
            predicted_label = chr(np.argmax(prediction) + ord('A'))
            clue += predicted_label

        clueType=""
        for idx, char_img in enumerate(clueChars):
            char_img = char_img.astype(np.float32) / 255.0
            char_img = cv2.cvtColor(char_img, cv2.COLOR_GRAY2RGB)
            char_img = np.expand_dims(char_img, axis=0)

            prediction = self.model.predict(char_img)[0]
            predicted_label = chr(np.argmax(prediction) + ord('A'))
            clueType += predicted_label

        #Publish if needed
        rospy.loginfo(f"Detected clue: {clue}")
        rospy.loginfo(f"Detected type: {clueType}")

        self.update_csv(clueType, clue)
    
if __name__ == '__main__':
    clueReader()
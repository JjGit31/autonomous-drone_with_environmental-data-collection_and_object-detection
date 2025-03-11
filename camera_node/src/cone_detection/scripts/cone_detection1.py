#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, NavSatFix
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Twist
import csv
import os

# Initialize ROS node
rospy.init_node("cone_detection_node")

# Create CvBridge for ROS to OpenCV conversion
bridge = CvBridge()

# Global variables
current_gps = None
cone_gps_locations = set()  # Use a set to avoid duplicates
cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
FRAME_CENTER = (320, 240)
THRESHOLD = 20
DEAD_ZONE = 10
GPS_CSV_FILE = "/home/Rpi/gps_locs.csv"

# Ensure CSV file exists and has a header
if not os.path.exists(GPS_CSV_FILE):
    with open(GPS_CSV_FILE, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Latitude", "Longitude"])  # Column headers

def gps_callback(msg):
    """Updates current GPS coordinates."""
    global current_gps
    current_gps = (msg.latitude, msg.longitude)

def move_drone(linear_x, linear_y, angular_z):
    """Publishes movement commands to the drone."""
    twist = Twist()
    twist.linear.x = linear_x
    twist.linear.y = linear_y
    twist.angular.z = angular_z
    cmd_vel_pub.publish(twist)

def save_cone_gps(gps_coordinates):
    """Saves detected cone GPS location to a CSV file."""
    if gps_coordinates in cone_gps_locations:
        return  # Avoid saving duplicates

    cone_gps_locations.add(gps_coordinates)  # Store in set

    with open(GPS_CSV_FILE, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([gps_coordinates[0], gps_coordinates[1]])

    print(f"Saved cone GPS location: {gps_coordinates}")

def image_callback(msg):
    """Processes the camera feed and detects cones."""
    global current_gps
    if current_gps is None:
        return

    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        return

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([0, 145, 80])
    upper_orange = np.array([225, 180, 130])
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    best_contour = None
    max_area = 0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 100 and area > max_area:
            max_area = area
            best_contour = cnt

    center_x, center_y = FRAME_CENTER
    if best_contour is not None:
        x, y, w, h = cv2.boundingRect(best_contour)
        center_x = int(x + w / 2)
        center_y = int(y + h / 2)

    move_x, move_y = 0, 0

    if abs(center_x - FRAME_CENTER[0]) > THRESHOLD:
        move_y = -0.1 if center_x < FRAME_CENTER[0] else 0.1

    if abs(center_y - FRAME_CENTER[1]) > THRESHOLD:
        move_x = -0.1 if center_y < FRAME_CENTER[1] else 0.1

    # If the cone is centered within the dead zone, log the GPS location
    if abs(center_x - FRAME_CENTER[0]) < DEAD_ZONE and abs(center_y - FRAME_CENTER[1]) < DEAD_ZONE:
        move_x, move_y = 0, 0
        save_cone_gps(current_gps)

    move_drone(move_x, move_y, 0)

rospy.Subscriber("/usb_cam_node/image_raw", Image, image_callback)
rospy.Subscriber("/gps/fix", NavSatFix, gps_callback)

rospy.spin()


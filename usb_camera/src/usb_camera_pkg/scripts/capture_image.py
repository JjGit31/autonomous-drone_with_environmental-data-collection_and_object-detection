#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

# Directory to save images
SAVE_DIR = "/home/Rpi/cone_images/"

# Ensure the directory exists
if not os.path.exists(SAVE_DIR):
    os.makedirs(SAVE_DIR)

def capture_image():
    # Initialize the ROS node
    rospy.init_node('image_capture_node', anonymous=True)

    # Initialize CvBridge to convert ROS image messages to OpenCV format
    bridge = CvBridge()

    # Generate a random filename using the current timestamp
    timestamp = time.strftime("%Y%m%d_%H%M%S")  # Format: YYYYMMDD_HHMMSS
    filename = "cone_" + timestamp + ".jpg"

    full_filename = os.path.join(SAVE_DIR, filename)

    # Create a callback function to handle the image message
    def image_callback(msg):
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            # Save the image to the specified file
            cv2.imwrite(full_filename, cv_image)
            rospy.loginfo(f"Image saved to {full_filename}")
        except Exception as e:
            rospy.logerr(f"Error capturing image: {e}")

    # Subscribe to the camera topic (e.g., /usb_cam/image_raw)
    rospy.Subscriber("/usb_cam_node/image_raw", Image, image_callback)

    # Spin to keep the node running and processing messages
    rospy.spin()

if __name__ == '__main__':
    # Run the capture_image function without requiring a filename argument
    capture_image()


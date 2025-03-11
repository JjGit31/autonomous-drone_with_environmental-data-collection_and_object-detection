#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
import datetime

# Initialize the ROS node
rospy.init_node("cone_detection_node")

# File to save the most recent coordinates and image
output_file = "/home/Rpi/new_coordinates_new.txt"  # Adjust path if needed
image_save_path = "/home/Rpi/cone_images"  # Directory to save cone images
os.makedirs(image_save_path, exist_ok=True)

# Create CvBridge for ROS to OpenCV image conversion
bridge = CvBridge()

# Global variable to store the latest GPS coordinates
current_gps = None

def gps_callback(msg):
    global current_gps
    # Store the latest GPS data (latitude, longitude)
    current_gps = (msg.latitude, msg.longitude)

def image_callback(msg):
    global current_gps

    if current_gps is None:
        rospy.logwarn("No GPS data available. Skipping image processing.")
        return

    try:
        # Convert ROS Image message to OpenCV image
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"CV Bridge Error: {e}")
        return

    # Convert frame to HSV for color detection
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define color range for orange cones (adjust as per your cone's color)
    lower_orange = np.array([25, 100, 100])  # Example: Hue range for orange
    upper_orange = np.array([50, 255, 255])

    # Create a mask for orange color
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        # Filter based on area to avoid small noise
        area = cv2.contourArea(cnt)
        if area > 100:  # Adjust threshold as needed
            # Get bounding box for the contour
            x, y, w, h = cv2.boundingRect(cnt)

            # Calculate center coordinates of the cone
            center_x = int(x + w / 2)
            center_y = int(y + h / 2)

            # Draw rectangle and center point on the frame
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

            # Overwrite the file with the most recent coordinates
            with open(output_file, "w") as f:
                f.write(f"Last Detected Coordinates: ({center_x}, {center_y})\n")
                if current_gps:
                    f.write(f"GPS Coordinates: ({current_gps[0]}, {current_gps[1]})\n")

            # Save the image with a unique filename
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            image_filename = os.path.join(image_save_path, f"cone_{timestamp}.jpg")
            cv2.imwrite(image_filename, frame)

            # Display coordinates on the frame
            cv2.putText(frame, f"({center_x}, {center_y})", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.putText(frame, f"GPS: ({current_gps[0]}, {current_gps[1]})", (x, y - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            rospy.loginfo(f"Detected Coordinates: ({center_x}, {center_y})")
            rospy.loginfo(f"GPS Coordinates: {current_gps}")

    # Display the frame
    cv2.imshow("Cone Detection", frame)

    # Handle GUI events and allow 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("User requested shutdown.")

# Subscribe to the camera topic
rospy.Subscriber("/usb_cam_node/image_raw", Image, image_callback)

# Subscribe to the GPS topic
rospy.Subscriber("/gps/fix", NavSatFix, gps_callback)

# Inform the user
rospy.loginfo("Cone detection node is running. Press 'q' in the display window to exit.")

# Keep the node alive
try:
    rospy.spin()
except KeyboardInterrupt:
    rospy.loginfo("Shutting down.")

# Release resources
cv2.destroyAllWindows()

#!/usr/bin/env python3

import time
import math
import subprocess
import datetime
import os
from dronekit import connect, VehicleMode, LocationGlobalRelative

# Connect to the Vehicle
connection_string = "/dev/ttyACM0"  # Adjust as needed
print(f"Connecting to vehicle on {connection_string}")
vehicle = connect(connection_string, wait_ready=True, baud=57600, heartbeat_timeout=60)

# Image storage path
image_save_path = "/home/Rpi/cone_images"
os.makedirs(image_save_path, exist_ok=True)

# Sensor scripts
temp_sensor_script = "/home/Rpi/sens_wks/src/temp/scripts/dht22.py"
gas_sensor_script = "/home/Rpi/mq135_ws/src/mq135_sensor/scripts/mq135_data.py"

# Function to arm and take off
def arm_and_takeoff(target_altitude):
    print("Arming the drone...")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        time.sleep(1)
    
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)
    
    print(f"Taking off to {target_altitude} meters...")
    vehicle.simple_takeoff(target_altitude)
    while vehicle.location.global_relative_frame.alt < target_altitude * 0.95:
        time.sleep(1)
    print("Reached target altitude!")

# Function to run cone detection script
def run_cone_detection():
    cone_detection_path = "/home/Rpi/camera_node/src/cone_detection/scripts/cone_detection1.py"
    print("Starting cone detection for local adjustment...")
    subprocess.run(["python3", cone_detection_path])
    print("Cone detection completed. Drone positioned.")

# Function to capture image
def capture_image():
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    image_filename = os.path.join(image_save_path, f"cone_{timestamp}.jpg")
    subprocess.run(["python3", "/home/Rpi/usb_camera_pkg/scripts/capture_image.py", image_filename])
    print(f"Captured image: {image_filename}")

# Function to collect sensor data
def collect_sensor_data():
    print("Collecting sensor data...")
    subprocess.run(["python3", temp_sensor_script])
    subprocess.run(["python3", gas_sensor_script])
    print("Sensor data collection complete.")

# Function to fly to a GPS location
def fly_to_location(latitude, longitude, altitude):
    vehicle.airspeed = 1  # Set airspeed to 1 m/s

    target_location = LocationGlobalRelative(latitude, longitude, altitude)
    vehicle.simple_goto(target_location)
    
    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        lat_diff = abs(vehicle.location.global_relative_frame.lat - latitude) * 111320
        lon_diff = abs(vehicle.location.global_relative_frame.lon - longitude) * 111320
        distance = math.sqrt(lat_diff**2 + lon_diff**2)
        
        if distance < 1:
            break
        time.sleep(2)
    
    print("Reached waypoint!")

# Function for landing
def land():
    print("Initiating landing procedure...")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.mode.name != "LAND":
        time.sleep(1)
    
    while vehicle.location.global_relative_frame.alt > 0.1:
        time.sleep(1)
    
    print("Landed successfully!")

# Waypoints
waypoints = [
    (15.4953307, 73.9474189, 3),
    (15.4952996, 73.9475571, 3),
    (15.4953849, 73.9476161, 3)
]

try:
    # Arm and take off
    arm_and_takeoff(3)

    # Fly to each waypoint, detect the cone, and collect data
    for idx, (lat, lon, alt) in enumerate(waypoints, start=1):
        print(f"Flying to Waypoint {idx}...")
        fly_to_location(lat, lon, alt)
        time.sleep(3)

        # Run cone detection script
        run_cone_detection()

        # Capture images at different altitudes
        for altitude in [3.0, 4.5, 6.0]:  # 10ft, 15ft, 20ft
            vehicle.simple_goto(LocationGlobalRelative(lat, lon, altitude))
            time.sleep(3)  # Allow time for drone to reach altitude
            capture_image()

        # Collect sensor data
        collect_sensor_data()
    
    # Land
    land()
    vehicle.armed = False

finally:
    print("Closing connection...")
    vehicle.close()


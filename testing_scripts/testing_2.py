from dronekit import connect, VehicleMode, LocationGlobalRelative
import subprocess
import time
import math

# Connect to the Vehicle
connection_string = '/dev/ttyACM0'  # Adjust as needed
print(f"Connecting to vehicle on {connection_string}")
vehicle = connect(connection_string, wait_ready=True, baud=57600, heartbeat_timeout=60)

# Function to read RC channel value
def read_rc_channel(channel):
    """Returns the RC channel value (1000 to 2000)."""
    return vehicle.channels.get(str(channel), 0)

# Wait for toggle switch (RC Channel 7) to be ON before starting
print("Waiting for RC toggle switch ON...")
while read_rc_channel(6) < 1500:
    time.sleep(0.5)

print("Toggle detected! Starting flight script...")

# Function to convert latitude/longitude differences to meters
def latlon_to_meters(lat1, lon1, lat2, lon2):
    R = 6371000  # Earth radius in meters
    lat_diff = (lat2 - lat1) * 111320
    lon_diff = (lon2 - lon1) * (111320 * math.cos(math.radians(lat1)))
    distance = math.sqrt(lat_diff**2 + lon_diff**2)
    return lat_diff, lon_diff, distance

# Function to check if GPS has a fix
def has_gps_fix():
    """Checks if the vehicle has a valid GPS fix."""
    gps = vehicle.gps_0
    if gps.fix_type >= 3:  # 3 or higher means 3D fix
        return True
    else:
        return False

# Function to wait for a valid GPS fix
def wait_for_gps():
    """Waits until the vehicle has a valid GPS fix."""
    print("Waiting for GPS fix...")
    while not has_gps_fix():
        time.sleep(1)
    print("GPS fix acquired!")

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
    
    time.sleep(3)

# Function to run the camera node and cone detection script
def run_cone_detection_script():
    print("Starting the camera node and cone detection script...")

    # Launch ROS camera node
    camera_launch_file = "usb_camera_pkg usb_camera_node.launch"
    camera_process = subprocess.Popen(["roslaunch"] + camera_launch_file.split(), stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    # Run cone detection script
    cone_detection_path = "/home/Rpi/camera_node/src/cone_detection/scripts/cone_detection.py"
    cone_process = subprocess.Popen(["python3", cone_detection_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    print("Camera node and cone detection script are running!")
    return camera_process, cone_process

# Function to stop the camera node and cone detection script
def stop_cone_detection_script(camera_process, cone_process):
    print("Stopping the camera node and cone detection script...")
    camera_process.terminate()
    cone_process.terminate()
    print("Camera node and cone detection script stopped!")

# Function to fly to a given GPS location
def fly_to_location(latitude, longitude, altitude):
    target_location = LocationGlobalRelative(latitude, longitude, altitude)
    vehicle.simple_goto(target_location)
    while True:
        lat_diff_m, lon_diff_m, total_distance_m = latlon_to_meters(
            vehicle.location.global_relative_frame.lat,
            vehicle.location.global_relative_frame.lon,
            latitude, longitude)
        if total_distance_m < 1:
            break
        time.sleep(2)
    print("Reached waypoint!")

# Function for landing the drone safely
def land():
    """Lands the drone safely."""
    print("Initiating landing procedure...")
    
    # Switch to LAND mode
    vehicle.mode = VehicleMode("LAND")
    while vehicle.mode.name != "LAND":
        print("Waiting for mode to switch to LAND...")
        time.sleep(1)
    
    # Wait until the vehicle is close to the ground (0.1 meters)
    while vehicle.location.global_relative_frame.alt > 0.1:
        print(f"Altitude: {vehicle.location.global_relative_frame.alt} meters, landing...")
        time.sleep(1)
    
    print("Landed successfully!")

# Main Script
try:
    # Start the camera node and cone detection script
    camera_process, cone_process = run_cone_detection_script()
    
    # Wait for GPS fix before arming
    wait_for_gps()

    # Arm and take off
    arm_and_takeoff(1)
    
    # Perform the landing using the new function
    land()
    
    # Disarm the drone
    vehicle.armed = False

finally:
    # Stop the camera node and cone detection script
    stop_cone_detection_script(camera_process, cone_process)
    
    print("Closing connection...")
    vehicle.close()


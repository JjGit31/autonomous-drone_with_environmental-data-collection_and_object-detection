from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative

import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = '/dev/ttyAMA0'
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

print('Connecting to vehicle on: %s' % connection_string)

# Try to connect to the vehicle
try:
    vehicle = connect(connection_string, wait_ready=True, timeout=60)  # Added timeout for debugging
    print("Connected successfully!")
except Exception as e:
    print(f"Connection failed: {e}")
    exit(1)

def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(5)

arm_and_takeoff(10)

print("Set default/target airspeed to 3")
vehicle.airspeed = 3

print("Going towards first point for 30 seconds ...")
point1 = LocationGlobalRelative(21.16666077573288, 72.7845377487673, 20)
vehicle.simple_goto(point1)
time.sleep(30)

print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")
point2 = LocationGlobalRelative(21.16708722034739, 72.78566137575419, 20)
vehicle.simple_goto(point2, groundspeed=10)
time.sleep(30)

print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")

print("Close vehicle object")
vehicle.close()

if sitl:
    sitl.stop()


from dronekit import connect, VehicleMode
import time

# Connect to the Vehicle
# Replace '127.0.0.1:14550' with your connection string if different (e.g., '/dev/ttyACM0' or IP)
connection_string = '/dev/ttyACM0'
print(f"Connecting to vehicle on {connection_string}")
#vehicle = connect(connection_string, wait_ready=True)
#vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=57600)
vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=57600, heartbeat_timeout=60)



# Function to arm the drone
def arm_drone():
    print("Arming the drone...")
    
    # Set the vehicle to GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.mode.name == "GUIDED":
        print("Waiting for GUIDED mode...")
        time.sleep(1)
    
    # Arm the vehicle
    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for the vehicle to arm...")
        time.sleep(1)
    
    print("Drone armed!")

# Function to disarm the drone
def disarm_drone():
    print("Disarming the drone...")
    vehicle.armed = False
    while vehicle.armed:
        print("Waiting for the vehicle to disarm...")
        time.sleep(1)
    
    print("Drone disarmed!")

# Main Script
try:
    # Arm the drone
    arm_drone()
    
    # Wait for 5 seconds while armed
    print("Drone is armed. Waiting for 5 seconds...")
    time.sleep(5)
    
    # Disarm the drone
    disarm_drone()

finally:
    # Close the vehicle connection
    print("Closing connection...")
    vehicle.close()
    print("Connection closed.")


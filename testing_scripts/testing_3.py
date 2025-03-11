from dronekit import connect, VehicleMode
import time

# Connect to the Vehicle (this can be a serial connection, UDP, or TCP connection)
# Replace 'udp:127.0.0.1:14550' with your actual connection string, such as '/dev/ttyUSB0' or 'udp:0.0.0.0:14550'
# If you're using a real drone, connect via a serial port like '/dev/ttyUSB0' or a network stream.
connection_string = "/dev/ttyACM0"  # Change to the correct serial port or network connection

# Connect to the vehicle
print(f"Connecting to vehicle at {connection_string}...")
vehicle = connect(connection_string, wait_ready=True)

# Function to arm the drone
def arm_drone():
    print("Arming drone...")
    # Check that vehicle is in a mode that allows arming
    if vehicle.is_armable:
        # Set the vehicle mode to GUIDED (or another mode that allows arming)
        vehicle.mode = VehicleMode("GUIDED")
        # Arm the vehicle
        vehicle.arm()
        while not vehicle.armed:
            print("Waiting for vehicle to arm...")
            time.sleep(1)
        print("Drone is armed!")
    else:
        print("Vehicle not armable")



# Function to takeoff to a specified altitude
def takeoff_drone(altitude):
    print(f"Taking off to {altitude} meters...")
    # Ensure the vehicle is in GUIDED mode
    if vehicle.mode.name != "GUIDED":
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)  # Give some time for mode change

    # Command the drone to take off to the desired altitude
    vehicle.simple_takeoff(altitude)
    
    # Wait until the vehicle reaches the target altitude
    while True:
        print(f"Current altitude: {vehicle.location.global_relative_frame.alt} meters")
        if vehicle.location.global_relative_frame.alt >= altitude * 0.95:  # 95% of target altitude
            print(f"Reached target altitude of {altitude} meters")
            break
        time.sleep(1)
        
# Function to land the drone safely
def land_drone():
    print("Initiating landing sequence...")
    # Ensure the vehicle is in GUIDED mode for landing
    if vehicle.mode.name != "GUIDED":
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)  # Give some time for mode change

    # Command the drone to land
    vehicle.simple_land()
    
    # Wait for the drone to land
    while vehicle.location.global_relative_frame.alt > 0.5:  # Wait until the altitude is below 0.5 meters
        print(f"Current altitude: {vehicle.location.global_relative_frame.alt} meters")
        time.sleep(1)
    
    print("Drone has landed!")
    # Function to disarm the drone
def disarm_drone():
    print("Disarming drone...")
    vehicle.mode = VehicleMode("STABILIZE")  # Change to a safe mode (e.g., STABILIZE)
    vehicle.disarm()
    while vehicle.armed:
        print("Waiting for vehicle to disarm...")
        time.sleep(1)
    print("Drone is disarmed!")

# Example usage
if __name__ == "__main__":
    try:
        # Arm the drone
        arm_drone()
        time.sleep(2)  # Wait for a few seconds

        # Takeoff to 10 meters
        takeoff_drone(2)

        time.sleep(3)  # Wait for a few seconds at altitude

        land_drone()

        # Disarm the drone
        disarm_drone()

    except KeyboardInterrupt:
        print("Program interrupted!")
    finally:
        # Close the connection to the vehicle
        print("Closing connection...")
        vehicle.close()


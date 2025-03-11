from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math

# Connect to Pixhawk
connection_string = '/dev/ttyACM0'  # Adjust as needed
vehicle = connect(connection_string, wait_ready=True, baud=57600)

# Function to read RC channel value
def read_rc_channel(channel):
    """Returns the RC channel value (1000 to 2000)."""
    return vehicle.channels.get(str(channel), 0)

# Wait for toggle switch (RC Channel 7) to be ON before starting
print("Waiting for RC toggle switch ON...")
while read_rc_channel(6) < 1500:
    time.sleep(0.5)

print("Toggle detected! Starting flight script...")

print("Closing connection...")
vehicle.close()

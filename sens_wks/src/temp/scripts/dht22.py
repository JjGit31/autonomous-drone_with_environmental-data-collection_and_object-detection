import sys
from time import sleep  # Import Sleep module from time library
import Adafruit_DHT     # Import the Adafruit_DHT module
import csv
from datetime import datetime  # For timestamping

pin = 4                 # Set pin to pin 4
dly = 2                 # Set delay to 2000ms (2 seconds) Can be changed to 1 for DHT22
sensor_type = 22        # Sensor type: Change this to 22 if using DHT22, or leave
                        # at 11 for DHT11

# Variables to track the total sum and count for averaging
total_temperature_c = 0.0
total_temperature_f = 0.0
total_humidity = 0.0
count = 0

# Open the CSV file in append mode to store the data
# This opens the file in append mode so it won't overwrite the file
with open('sensor_data.csv', mode='a', newline='') as file:
    writer = csv.writer(file)

    # Write header only if the file is empty (using the tell() function to check if the file has content)
    if file.tell() == 0:
        writer.writerow(['Timestamp', 'Temperature (C)', 'Temperature (F)', 'Humidity (%)', 'Average Temperature (C)', 'Average Temperature (F)', 'Average Humidity (%)'])

    try:
        while True:
            # Introduce our delay
            sleep(dly)

            # Read from sensor
            humidity, temperature = Adafruit_DHT.read_retry(sensor_type, pin)

            # Check if data from sensor read is valid
            if humidity is not None and temperature is not None:
                # Apply calibration: subtract 3°C for calibration
                calibrated_celsius = temperature  # Subtract 3°C for calibration
                fahrenheit = (calibrated_celsius * 1.8) + 32  # Convert to Fahrenheit
                celsius = calibrated_celsius  # Use the calibrated Celsius value

                # Get current timestamp
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

                # Update the total sum and count
                total_temperature_c += celsius
                total_temperature_f += fahrenheit
                total_humidity += humidity
                count += 1

                # Calculate averages
                avg_temp_c = total_temperature_c / count
                avg_temp_f = total_temperature_f / count
                avg_humidity = total_humidity / count

                # Print the calibrated data and averages to the console with 6 decimal places
                print(f'Temperature: ({celsius:0.6f}C) ({fahrenheit:0.6f}F) Humidity: {humidity:0.1f}%')
                print(f"Average Temperature: ({avg_temp_c:0.6f}C) ({avg_temp_f:0.6f}F), Average Humidity: {avg_humidity:0.1f}%")

                # Write data and averages to the CSV file with 6 decimal places for temperature
                writer.writerow([timestamp, f"{celsius:0.6f}", f"{fahrenheit:0.6f}", f"{humidity:0.1f}", f"{avg_temp_c:0.6f}", f"{avg_temp_f:0.6f}", f"{avg_humidity:0.1f}"])

            else:
                print('Cannot read from device')

    except KeyboardInterrupt:
        print("Program interrupted. Exiting...")
        sys.exit()


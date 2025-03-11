#!/usr/bin/env python3

import spidev
import time
import math
import csv

# Setup SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # bus 0, device 0 (CE0)
spi.max_speed_hz = 50000

# Function to read the ADC value
def read_adc(channel):
    if channel < 0 or channel > 7:
        return -1
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((adc[1] & 3) << 8) + adc[2]
    return data

# Function to calculate gas concentration (dummy calibration values)
# RO is the sensor resistance in clean air, and the factor is for a specific gas
def calculate_concentration(resistance, ro, factor):
    return factor * math.pow((resistance / ro), -1)

# Function to save data to a CSV file (appends data)
def save_to_csv(data):
    with open('sensor_data.csv', mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(data)

# Function to write the header to the CSV file (if the file is empty)
def write_header():
    try:
        # Open the file in read mode to check if it's empty
        with open('sensor_data.csv', mode='r') as file:
            if file.tell() == 0:  # If the file is empty, write the header
                writer = csv.writer(file)
                writer.writerow(['Timestamp', 'CO2 Concentration (ppm)', 'NH3 Concentration (ppm)', 'Benzene Concentration (ppm)', 'Alcohol Concentration (ppm)', 'Smoke Concentration (ppm)'])
    except FileNotFoundError:
        # If the file doesn't exist, create it and write the header
        with open('sensor_data.csv', mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Timestamp', 'CO2 Concentration (ppm)', 'NH3 Concentration (ppm)', 'Benzene Concentration (ppm)', 'Alcohol Concentration (ppm)', 'Smoke Concentration (ppm)'])

# Main loop to read sensor and calculate concentrations for different gases
def read_sensor():
    try:
        # Write the header to CSV if it's not already written
        write_header()

        while True:
            # Read raw value from MQ-135 sensor (channel 0)
            sensor_value = read_adc(0)

            # Convert raw value to sensor resistance (assuming 5V and using a simple formula)
            Vout = (sensor_value / 1024.0) * 5.0  # Convert ADC to voltage
            Rload = 10  # Load resistor value, adjust according to your circuit
            Rs = (5.0 - Vout) / Vout * Rload  # Sensor resistance

            # RO is the sensor resistance in clean air, this will vary based on calibration
            RO = 10  # For example, the clean air resistance for the MQ-135

            # Dummy calibration factors for different gases (replace with actual calibration factors)
            CO2_factor = 1.2
            NH3_factor = 1.5
            Benzene_factor = 1.8
            Alcohol_factor = 2.0
            Smoke_factor = 2.5

            # Calculate gas concentrations
            CO2_concentration = calculate_concentration(Rs, RO, CO2_factor)
            NH3_concentration = calculate_concentration(Rs, RO, NH3_factor)
            Benzene_concentration = calculate_concentration(Rs, RO, Benzene_factor)
            Alcohol_concentration = calculate_concentration(Rs, RO, Alcohol_factor)
            Smoke_concentration = calculate_concentration(Rs, RO, Smoke_factor)

            # Get the current timestamp
            timestamp = time.strftime('%Y-%m-%d %H:%M:%S')

            # Print the concentrations (in ppm or other appropriate units)
            print(f"Timestamp: {timestamp}")
            print(f"CO2 Concentration: {CO2_concentration:.2f} ppm")
            print(f"NH3 Concentration: {NH3_concentration:.2f} ppm")
            print(f"Benzene Concentration: {Benzene_concentration:.2f} ppm")
            print(f"Alcohol Concentration: {Alcohol_concentration:.2f} ppm")
            print(f"Smoke Concentration: {Smoke_concentration:.2f} ppm")
            print("-----------------------------------------------------------")

            # Save the data to CSV
            save_to_csv([timestamp, CO2_concentration, NH3_concentration, Benzene_concentration, Alcohol_concentration, Smoke_concentration])

            # Sleep for a second before the next reading
            time.sleep(1)

    except KeyboardInterrupt:
        print("Program terminated.")
    finally:
        spi.close()

# Run the sensor reading function
read_sensor()


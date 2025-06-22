# Autonomous Drone made for Object Detection and Environmental Data Collection

The repo contains ROS packages, Python Scripts for Sensors and Dronekit Scripts for drone operation and navigation and for mission planning.
Initial setup of the companion computer Raspi 4 used in the project is the setup and installation of ubuntu(20.04 recommended) using raspi imager and operating it via SSH.
Enable the camera and communication protocols (SPI and UART) through GPIO pins by installing raspi-config if not already present.
Install ROS 1 noetic, python3 and other necessary necessary requirements before cloning the repo.

Devices/components used in this project are are follows:

  - Pixhawk Flight Controller 2.4.8 
  - Raspberry PI 4
  - Raspi CSI Camera
  - MQ135(Gas composition detection)
  - DTH22(temperature and Humidity)
  - Drone Radio Telemetry Module   
  - Brushless Drone Motors 1000kv
  - Brushless Motor ESCs
  - Wifi Transceiver
  - Universal RC Controller and Receiver for Drones
     
## Pixhawk Flight Controller 2.4.8
The Pixhawk 2.4.8 is an open-source, advanced flight controller used primarily in drones (UAVs), rovers, boats, and other autonomous vehicles. 
It is compatible with ArduPilot and PX4 autopilot firmware.
I2C and SPI ports: For external sensors like rangefinders, GPS, or magnetometers.
Telemetry ports (TELEM1, TELEM2): For real-time communication with ground control stations.
USB port: For firmware upload and configuration via Mission Planner/QGroundControl.

### Ground Control Software
- Mission Planner (ArduPilot)
- QGroundControl (PX4 & ArduPilot)
#### In this project Mission Planner was used as a ground control and monitoring software and can be Interfaced with the Pixhawk via MAVLink (through telemetry or USB).

## Raspberry PI 4 ##
Single-board computer used as the main onboard companion computer in the Drone 
Raspberry Pi 4 is used as a co-processing device for guiding and navigation of drone.For this Dronekit was used for mission planning and navigation of Drone.
Ubuntu 20.04 with ROS(Robot Operating System) 1 Noetic was intalled on the raspi for enabling high-level tasks like:

  - Object detection-For detecting a Orange Cone in this case
  - Data processing and storing- GPS coordinates and logs etc.
  - Guiding and Navigation-Using a Python API and is used it to send commands, read telemetry, and run and plan flight missions.
  - Sensor Data Collection and storage-Sensor data are read and stored in a text/csv file in the onboard-raspi.

#### Raspi CSI Camera : 
Connected to the Pi via the CSI (Camera Serial Interface) port and used for real-time video capture and processing the video data using computer vision tasks using OpenCV 
and for object detection (e.g., orange colour cones for this project) YOLO v5 is used for better real-time detection.
Lightweight and optimized for Pi, with low latency without any extra workload for interfacing and is optimized to use with raspi.

#### MQ135 (Gas composition detection) Sensor: 
Can be used for Detecting Air Quality and Gas Concentrations (e.g., CO₂, NH₃, benzene, smoke).
It was used in Drone for environmental monitoring.
Outputs analog/digital signal processed by Pi or microcontroller

#### DTH22 (temperature and Humidity) sensor: 
It was used in drone for Atmospheric condition logging that is for Temp and Humidity and can be used for Weather-sensitive applications
Interfaces via GPIO, communicates using a simple 1-wire protocol which is Timing-sensitive: the Raspberry Pi reads the pulse lengths to determine temperature and humidity 
and Libraries like Adafruit_DHT handle this timing and decoding in software.

#### Radio Telemetry Module for Drone:  
Allowed wireless data communication between the drone and ground control station for Real-time telemetry (position, speed, battery etc information) 
and can be used for Mission planning & monitoring using Mission Planner.

#### Brushless Drone Motors 1000kv: 
Using powerful motor is suggested as it was noticed that using the drone manully with rc controller generated more power than autonomously controlled drone (Height observed was significantly less than flying it manuallly).
Propellers size depends upon the drone's weight and dimentions and the motor capacity to generate sufficient thrust.

#### Brushless Motor ESCs: 
Act as the interface between battery and motors and Translate PWM signals from Pixhawk to control motor speed.

#### Wifi Transceiver Module: 
Optional for extending the Wireless connection between the Ground-Station device and the on-board Raspi for enabling the SSH connection.

#### Universal RC Controller and Transceiver :
For manual control of the drone and is necessary for Flight operation.Configure the Flight Modes as the drone can work Autonomous only in Guided Mode.

### ROS Framework usage
ROS 1 noetic is used in this project for synchronous sensor data integration and camera feed processing (the ros framework can be used for further improvement of the system if it is fully ROS operated).
ROS nodes read data from MQ135 (gas) and DHT22 (temp/humidity), publish on topics or store in a file locally.
Here ROS was mainly used to isolate and for reusablility of each function (camera, detection, sensor, logic etc) 
If integrated with MAVROS, ROS could also send navigation commands to Pixhawk via MAVLink (optional).
ROS publisher and subscriber was used to share data between camera and decision logic present in the cone detection script in which x and y coordinates marker which were found after the detection, were used 
to create a mapping logic so that the drone, once arrives near the approximate gps coordinates location then it can navigate itself over the cone using that mapping logic).

## Physical Connections between systems:
For connection between pixhawk and raspi you can do it either with usb cable (pixhawk has a usb port for input power and connection) or by telemetry port which uses UART communication protocol.
For physical connections between the sensors and the raspi you can see the gpio pinout diagram and accordingly do the connection to the sensors.
Also the MQ135 produces an analog voltage based on gas concentration and the Raspberry Pi does NOT have a built-in ADC, so to read analog values, you need an external ADC chip like:
- MCP3008 (SPI)
- ADS1115 (I2C)

# Workspaces, Packages and Scripts structure and Overview:
#### Using Dronekit (Python compatible API for MAVlink) 
The main script for mission planning is the dronekit_final.py file present in the main directory along with the workspace mentioned 
this file contains the mission where gps coordinates and flying altitude can be inserted to first start the drone 
and the cone detection script will automatically start and the local mapping logic is present in the same file of cone detection.

usb_camera workspace contains a launch file which is used for launching camera feed publisher to a rostopic which can be seen by listing the rostopic once it has been started.

camera_node workspace contains the cone detection scripts that can be run after the ros node is publishing data on the topic which are subscribed by the cone detection script.The script also saves the x and y markers coordinates found after the detection to a coordinates text file and contains loacal mapping logic.

The python scripts which contains sensor libraries for data collection from sensors and for operating sensors are present in the mq135_ws workspace for MQ135 sensor and in sens_wks workspace for DHT22 sensor.The data is then logged in the text/csv file.

Some additional testing scripts are present inside the tesing_scripts workspace for tesing the drone (be careful and aware about for what function the particular testing scripts are for)

roslaunch command can be used to run this launch file and for running python scripts python3 can be used.

Additional scripts are present like focus detect for camera focus adjustment,shortest path script,zig zag script,teleop for operating the drone with tele keys,toggle for testing the operation(arming) of drone using the rc radio controller button which can be set to different button.

The gitignore file is for ignoring the ros packages/bundles which are downloaded with the ROS itself.





### Club Affiliation
🔧 Developed as part of the Team Aero Drishti, Sardar Vallabhbhai National Institute of Technology Surat
### Author 
Jitesh Jain:  https://github.com/JjGit31

🔗  LinkedIn: [profile](https://www.linkedin.com/in/jitesh-jain-06a3b5343?utm_source=share&utm_campaign=share_via&utm_content=profile&utm_medium=android_app)

exit
cd YDLidar-SDK
cd python
cd examples
ls
gedit plot_ydlidar_test.py
ls
python plot_ydlidar_test.py
import ydlidar
python ydlidar_test.py
ls
mv plot_ydlidar_test.py
mv plot_ydlidar_test.py ydlidar_test.py
ls
python3 ydlidar_test.py
gedit ydlidar_test.py
pip3 install numpy==1.19.5
python3 ydlidar_test.py
ls /dev/ttyUSB*
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0
python3 ydlidar_test.py
dmesg | grep tty
sudo chmod 666 /dev/ttyUSB0
sudo usermod -a -G dialout $USER
gedit ydlidar_test.py
python3 ydlidar_test.py
dmesg | tail -20
python3 ydlidar_test.py
gedit ydlidar_test.py
python3 ydlidar_test.py
still giving timeout count: 1
sudo apt install minicom
minicom -D /dev/ttyUSB0 -b 128000
sudo reboot
cd YDLidar-SDK
ls
cd buid
ls
cd python
ls
cd examples
ls
cd ..
mv buid build
ls
exit
ls 
cd YDLidar-SDK/
ls 
build 
cd build 
ls 
exit 
ls 
cd YDLidar-SDK/
ls 
cd build 
ls 
cd ..
clear 
exit 
cd YDLidar-SDK
ls
cd build
cmake ..
make
sudo make install
cd ..
pip install
pip install .
ls
cd build
ls
cd python
ls
cd examples
ls
cd ..
cd..
cd ..
cd buid
ls
cd python
ls
cd examples
ls
cd
sudo apt-get install -y git
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
sudo apt install cmake pkg-config
sudo apt-get install python swig
sudo apt-get install python pip
exit
roscore
sudo pip install dronekit
sudo pip install dronekit-sitl
sudo pip install mavproxy
sudo raspi-config 
udo nano /boot/config.txt
sudo nano /boot/config.txt
sudo pip install dronekit
sudo pip install mavproxy
sudo pip install dronekit-sitl
exit
ls /dev/serial*
sudo chmod 666 /dev/ttyUSB0
ls /dev/serial*
sudo chmod 666 /dev/ttyUSB0
lsusb
dmesg | grep tty
sudo apt install minicom
minicom -D /dev/serial0 -b 57600
minicom -D /dev/ttyAM0 -b 57600
minicom -D /dev/ttyAMA0 -b 57600
sudo apt install minicom
minicom -D /dev/serial0 -b 57600
minicom -D /dev/ttyAMA0 -b 57600
ls /dev/serial/by-id
ls /dev/serial/by-id /dev/serial/by-path
ls /dev/serial*
reboot
sudo reboot
ls /dev/tty*
sudo nano /boot/config.txt
sudo apt install udev
sudo ln -s /dev/ttyAMA0 /dev/serial/by-id/ttyAMA0
sudo ln -s /dev/ttyAMA0 /dev/serial/by-path/ttyAMA0
sudo systemctl status udev
sudo mkdir -p /dev/serial/by-id
sudo mkdir -p /dev/serial/by-path
sudo ln -s /dev/ttyAMA0 /dev/serial/by-id/ttyAMA0
sudo ln -s /dev/ttyAMA0 /dev/serial/by-path/ttyAMA0
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/serial/by-id
ls -l /dev/serial/by-path
sudo systemctl enable udev
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --aircraft MyCopter
sudo apt install screen
screen /dev/ttyAMA0 57600
ls /dev/serial*
ls /dev/serial/by-id /dev/serial/by-path
dmesg | grep tty
sudo usermod -a -G dialout Rpi
ls /dev/serial/by-id
ls /dev/serial/by-path
dmesg | grep tty
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --aircraft MyCopter
WARNING: You should uninstall ModemManager as it conflicts with APM and Pixhawk
Connect /dev/ttyAMA0 source_system=255
Failed to connect to /dev/ttyAMA0 : [Errno 16] could not open port /dev/ttyAMA0: [Errno 16] Device or resource busy: '/dev/ttyAMA0'
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --aircraft MyCopter
sudo usermod -aG dialout $USER
sudo usermod -aG dialout Rpi
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --aircraft MyCopter
lsof /dev/ttyAMA0
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --mav10
sudo systemctl stop ModemManager
sudo systemctl disable ModemManager
sudo systemctl stop ModemManager
sudo systemctl disable ModemManager
sudo systemctl stop ModemManager
sudo systemctl disable ModemManager
mavproxy.py --master=/dev/ttyAMA0 --baudrate 115200 --aircraft MyCopter
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600
stty -F /dev/ttyAMA0
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --aircraft MyCopter
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 -- MyCopter
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --sitl --aircraft MyCopter6
ls -l /dev/serial/by-id
ls -l /dev/serial/by-path
sudo apt-get install python3-pip python3-dev python3-serial
ls /dev/ttyAMA0
ls
gedit pymavlink_test.py 
python3 pymavlink_test.py 
gedit hb_request.py
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --aircraft MyCopter
python3 pymavlink_test.py 
python3 hb_request.py 
MyCopter/logs/
cd MyCopter/logs/
ls
cd 2025-01-20
ls
cd flight1/
ls
cat flight.tlog
cat flight.log
cat flight.tlog
less flight.tlog
cat flight.tlog.raw
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --logfile flight.tlog
cat flight.tlog
cd ../..
ls
cd 2025-01-19
ls
cd flight10
ls
cat flight.tlog
cat flight.tlog.raw
cat flight.tlog
less flight.tlog
cd ..
ls
cd flight1
ls
cata flight.tlog
cat flight.tlog
exit
cat /dev/ttyAMA0
stty -F /dev/ttyAMA0
ls -l /dev/serial*
stty -F /dev/ttyAMA0
sudo lsof /dev/ttyAMA0
sudo fuser /dev/ttyAMA0
stty -F /dev/ttyAMA0 57600
sudo kill -9 <PID>
sudo kill -9 1975
stty -F /dev/ttyAMA0 57600
sudo fuser /dev/ttyAMA0
stty -F /dev/ttyAMA0 57600stty -F /dev/ttyAMA0
stty -F /dev/ttyAMA0
cat /dev/ttyAMA0
sudo apt install screen
screen /dev/ttyAMA0 57600
screen -ls
sudo lsof /dev/ttyAMA0
sudo kill -9 <PID>
sudo kill -9 2321
sudo systemctl stop ModemManager
sudo systemctl disable ModemManager
echo $USER
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --mav10
screen /dev/ttyAMA0 57600
sudo lsof /dev/ttyAMA0
sudo kill -9 2640
sudo kill -9 2661
lsof /dev/ttyAMA0
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --mav10
MAVLink Protocol Version 2
sudo apt-get update
sudo apt-get install python3-pip
pip3 install MAVProxy
pip3 install pymavlink
mavproxy.py --version
gedit pymavlink_test.py
python3 pymavlink_test.py 
gedit pymavlink_test.py
python3 pymavlink_test.py 
exit
python3 mavproxy.py --master=/dev/ttyAMA0 --baudrate 921600 --aircraft MyCopter
ls
python3 mavproxy.py --master=/dev/serial0 --baudrate 921600 --aircraft MyCopter
python3 mavproxy.py --master=/dev/ttyAMA0 --baudrate 921600 --aircraft MyCopter
mavproxy.py --master=/dev/ttyAMA0 --baudrate 921600 --aircraft MyCopter
mavproxy.py --daemon --non-interactive --default-modules='' --continue --master=/dev/serial0 --baudrate 57600 --out=udp:pro:14550
mavproxy.py --daemon --non-interactive --default-modules='' --continue --master=/dev/serial0 --baudrate 921600 --out=udp:pro:14550
mavproxy.py --daemon --non-interactive --default-modules='' --continue --master=/dev/ttyAMA0 --baudrate 57600 --out=udp:pro:14550
mavproxy.py --daemon --non-interactive --default-modules='' --continue --master=/dev/ttyAMA0 --baudrate 921600 --out=udp:pro:14550
mavproxy.py --daemon --non-interactive --default-modules='' --continue --master=/dev/ttyAMA0 --baudrate 1500000 --out=udp:pro:14550
exit
mavproxy.py --master=/dev/ttyAMA0
sudo nano /boot/config.txt
sudo reboot
python3 mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --aircraft MyCopter
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --aircraft MyCopter
sudo nano /boot/config.txt
mavproxy.py --master=/dev/ttyAMAO
mavproxy.py --master=/dev/ttyAMA0
mavproxy.py --master=/dev/ttyUSB0
ls /dev/ttyUSB*
ls /dev/ttyUSB0
exit 
sudo pip install dronekit
sudo pip install dronekit-sitl
sudo pip install mavproxy
sudo reboot
sudo apt update
sudo apt install python3-pip
sudo pip3 install mavproxy
ls /dev/tty*
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600
sudo systemctl stop ModemManager
sudo systemctl disable ModemManager
sudo apt-get remove --purge modemmanager
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600
sudo usermod -aG dialout Rpi
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600
sudo lsof /dev/ttyAMA0
pgrep -af mavproxy
dmesg | grep tty
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --debug
sudo nano /boot/config.txt
sudo reboot
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --debug
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --debug
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --verbose
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --console
cd ws
cd devel
ls
cd src
ls
cd ../..
ls
cd bmp280
ls
cd devel
ls
sudo pip install bmp280
cd bmp280-python
sudo raspi-config
sudo reboot
PWD
pwd
sudo apt-get update
LS
ls
cd bmp280
cd src
ls
cd bmp280_ros
cd
cd bmp280
cd src
cd bmp280_ros
cd scripts
ls
gedit bmp280_data.py
cd
cd bmp280
cd devel
pwd
cd ..
catkin_make
cd
sudo pip3 install adafruit-circuitpython-bmp280
cd bmp280
source /home/Rpi/bmp280/devel/setup.bash
cd bmp280
cd src
cd bmp280_ros
cd scripts 
python3 bmp280_data.py
i2cdetect -y 1
cd bmp280
cd src
cd bmp280_ros
cd scripts
python3 bmp280_data.py
cd 
pip install --upgrade adafruit-circuitpython-bmp280
pip install --upgrade adafruit-circuitpython-busdevice
cd bmp280
cd src
cd bmp280_ros
cd scripts 
gedit bmp280_data.py
cd
sudo i2cdetect -y 1
cd bmp280
catkin_make
cd devel
pwd
cd
source /home/Rpi/bmp280/devel
source /home/Rpi/bmp280/devel/setup.bash
cd bmp280
cd src 
cd bmp280_ros
cd scripts
python3 bmp280_data.py
gedit bmp280_data.py
cd 
vim bmp280.py
ROSCORE
roscore
vim bmp280_data.py
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --console
dmesg | grep tty
sudo raspi-config
sudo reboot
cd bmp280
cd src
cd bmp280_ros
cd scripts
python3 bmp280_data.py
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --mavlink 2
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --console
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --mavlink 1
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --console
pip install mavproxy --upgrade
sudo apt install minicom
File "bmp280_data.py", line 5, in <module>
ModuleNotFoundError: No module named 'smbus'
sudo apt update
sudo apt install python3-smbus
i2cdetect -y 1
cd bmp280
cd src
cd ..
catkin_make
source home/bmp280/devel/setup.bash
cd devel
pwd
cd 
source /home/Rpi/bmp280/devel/setup.bash
cd bmp280
cd src
cd bmp280_ros
cd scripts
python3 bmp280_data,py
python3 bmp280_data.py
gedit bmp280_data.py
cd
cd bmp280
catkin_make
source home/bmp280/devel/setup.bash
source /home/Rpi/bmp280/devel/setup.bash
cd src
cd bmp280_ros
cd scripts
python3 bmp280_data.py
cd
sudo apt update
sudo apt upgrade
python3 bmp280_data.py
cd bmp280
cd src
cd bmp280_ros
cd scripts
python3 bmp280_data.py
cd
sudo pip install bmp280
git clone https://github.com/pimoroni/bmp280-python
roscore
cd ~
mkdir valt
cd valt
touch sensor.py
nano sensor.py
python3 sensor.py
touch sensor.pygedit sensor.py
gedit sensor.py
sudo usermod -aG dialout Rpi
mavproxy.py --master=/dev/ttyACM0 --baudrate 57600
exit 
sudo usermod -aG dialout Rpi
mavproxy.py --master=/dev/ttyACM0 --baudrate 57600
sudo usermod -aG dialout Rpi
mavproxy.py --master=/dev/ttyACM0 --baudrate 57600
ls /dev/tty
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600
mavproxy.py --master=/dev/ttyACM1 --baudrate 57600
ls

mavproxy.py --master=/dev/ttyACM1 --baudrate 57600
ls /dev/tty
mavproxy.py --master=/dev/ttyACM1 --baudrate 57600
sudo usermod -aG dialout Rpi
mavproxy.py --master=/dev/ttyACM1 --baudrate 57600
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600
exit 
i2cdetect -y 1
sudo apt update
sudo apt install -y i2c-tools
all -y python3-pip
sudo apt install -y python3-pip
sudo pip3 install adafruit-circuitpython-bmp280
sudo apt install -y python3-rosdep python3-catkin-tools python3-rospy
sudo apt install -y python3-pandas
cd bmp280
cd src
cd bmp280_ros
cd scripts
gedit bmmp280_data.py
cd bmp280/src/bmp280_ros/scripts
gedit bmp280_data.py
chmod +x scripts/bmp280_data.py
chmod +x ./bmp280_data_collector.py
chmod +x ./bmp280_data.py
cd
cd bmp280
catkin_make
cd devel
pwd
cd 
source /home/Rpi/bmp280/devel
source  /home/Rpi/bmp280/devel/setup.bash
cd bmp280/src/bmp280_ros/scripts
python3 bmp280_data.py
cd
pip3 install adafruit-blinka adafruit-circuitpython-bmp280
sudo raspi-config
sudo reboot
cd bmp280/src/bmp280_ros/scripts
python3 bmp280_data.py
cd
sudo nano /boot/config.txt
sudo python3 bmp280_data.py
cd bmp280/src/bmp280_ros/scripts
sudo python3 bmp280_data.py
sudo apt-get update
sudo apt-get install ros-noetic-sensor-msgs
sudo apt remove
source /opt/ros/noetic/setup.bash
sudo apt-get install python3-rospy
source ~/catkin_ws/devel/setup.bash
sudo python3 bmp280_data.py
rosrun bmp280_sensor bmp280_data_collector.py
rosrun bmp280_ros bmp280_data.py
sudo python3 bmp280_data.py
cd
ls
cd ~/catkin_ws
cd src
ls
cd mq135_sensor
ls
cd ..
catkin_create_pkg my_package rospy std_msgs
ls
catkin_create_pkg bmp280_sensor rospy std_msgs
ls
cd bmp280_sensor
ls
cd package.xml
gedit package.xml
cd
cd catkin_ws
catkin_make
cd
cd bmp280
catkin_make
source /home/bmp280/devel/setup.bash
cd devel
pwd
<build_depend>rospy</build_depend>
<build_depend>sensor_msgs</build_depend>
<exec_depend>rospy</exec_depend>
/home/Rpi/bmp280/devel
cd
/home/Rpi/bmp280/devel
source /home/Rpi/bmp280/devel/setup.bash
cd bmp280/src/bmp280_ros/scripts
sudo python3 bmp280_data.py
sudo apt-get update
sudo apt-get install ros-noetic-sensor-msgs
cd
sudo apt-get install ros-noetic-sensor-msgs
source ~/catkin_ws/devel/setup.bash
rospack list | grep sensor_msgs
sudo apt-get install python3-rospy
cd bmp280/src/bmp280_ros/scripts
sudo python3 bmp280_data.py
gedit bmp280_data.py
cd
sudo apt update
sudo apt-get install ros-noetic-sensor-msgs
python3 --version
rospack list | grep sensor_msgs
pip3 install adafruit-circuitpython-bmp280
cd catkin_ws/src/
ls
cd bmp280_sensor/
ls
nano package.xml 
cd
cd bmp280/src/bmp280_ros/scripts
gedit bmp280_data.py
cd
catkin_ws
cd catkin_ws
catkin_make
cd
source /home/rpi/devel/setup.bash
source /home/Rpi/devel/setup.bash
cd bmp280
cd devel
pwd
source source /home/rpi/devel/setup.bash
source /home/Rpi/devel/setup.bash
cd bmp280
pip3 install sensor_msgs
cd src
ls
cd bmp280_ros
ls
gedit package.xml
cd 
cd bmp280
catkin_make
cd devel
pwd
cd
source /home/Rpi/bmp280/devel/setup.bash
cd bmp280/src/bmp280_ros/scripts
sudo python3 bmp280_data.py
cd 
cd bmp280
sudo apt-get install ros-noetic-sensor-msgs
sudo apt-get install python3-rospy
sudo apt-get install ros-noetic-std-msgs
source /home/Rpi/bmp280/devel/setup.bash
cd src
cd bmp280/src/bmp280_ros/scripts
sudo python3 bmp280_data.py
cd bmp280/src/bmp280_ros
ls
gedit package.xml
cd scripts
gedit bmp280_data.py
sudo python3 bmp280_data.py
python3 bmp280_data.py
gedit bmp280_data.py
python3 bmp280_data.py
gedit bmp280-python
gedit bmp280-python.py
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        sudo usermod -aG dialout Rpi
The error occurs because the SSH host key for the Raspberry Pi has changed since the last time you connected to it. This often happens if the Raspberry Pi has been reinstalled, its SSH keys have been reset, or its IP address was reused by another device.
Here’s how to resolve it:
---
### 1. **Remove the Old Host Key**
The message provides a specific command to remove the offending key:
```bash
ssh-keygen -f "/home/tonystark/.ssh/known_hosts" -R "192.168.139.75"
```
mavproxy.py --master=/dev/ttyACM0 --baudrate 57600
This removes the old key associated with `192.168.139.75` from the `known_hosts` file.
---
### 2. **Reconnect to the Raspberry Pi**
Try connecting again:
```bash
ssh -X Rpi@192.168.139.75
```
ls
gedit dronekit_testing.py
from dronekit import connect, VehicleMode
import time
# Connect to the Vehicle
# Replace '127.0.0.1:14550' with your connection string if different (e.g., '/dev/ttyACM0' or IP)
connection_string = '127.0.0.1:14550'
print(f"Connecting to vehicle on {connection_string}")
vehicle = connect(connection_string, wait_ready=True)
# Function to arm the drone
def arm_drone():
# Function to disarm the drone
def disarm_drone():
# Main Script
try:
python3 dronekit_testing.py 
gedit dronekit_testing.py 
python3 dronekit_testing.py 
sudo usermod -aG dialout Rpi
gedit dronekit_testing.py 
sudo usermod -aG dialout Rpi
python3 dronekit_testing.py 
gedit dronekit_testing.py 
python3 dronekit_testing.py 
sudo usermod -aG dialout Rpi
python3 dronekit_testing.py 
gedit drone_script_test.py
ls
python3 dronekit_test.py
gedit drone_script_test.py
python3 dronekit_script_test.py
python3 drone_script_test.py 
gedit drone_script_test.py
python3 drone_script_test.py 
sudo usermod -aG dialout Rpi
mavproxy.py --master=/dev/ttyACM0 --baudrate 57600
sudo usermod -aG dialout Rpi
mavproxy.py --master=/dev/ttyACM0 --baudrate 57600
sudo usermod -aG dialout Rpi
mavproxy.py --master=/dev/ttyACM0 --baudrate 57600
exit
sudo raspi-config
sudo apt install i2c-tools
sudo i2cdetect -y 1
vim bmp280.py
sudo apt-get update
sudo apt-get install python3-pip
pip3 install pymavlink
ls
python3 drone_script_test.py 
gedit drone_script_test.py 
from pymavlink import mavutil
import time
# Define the serial connection to the Pixhawk (or flight controller)
# Replace '/dev/ttyUSB0' with the correct serial port and 57600 with the correct baud rate.
# Ensure the connection string matches the one you're using for your setup.
master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
# Wait for the heartbeat message to confirm communication
master.wait_heartbeat() print(f"Heartbeat from system (system ID: {master.target_system}, component ID: {master.target_component})")
def arm_drone():
gedit drone_script_test.py 
exit
vim bmp280_data.py
gedit drone_script_test.py 
sudo nano /etc/ssh/sshd_config
gedit drone_script_test.py 
nano drone_script_test.py 
sudo reboot
python3 drone_script_test.py 
ls
gedit dronekit_testing.py
exit
chmod +x drone_script_test.py
gedit drone_script_test.py 
chmod +x drone_script_test.py
mavproxy.py --master=/dev/ttyACM0 --baudrate 57600
dmesg | grep tty
chmod +x drone_script_test.py
mavproxy.py --master=/dev/ttyACM0 --baudrate 57600
chmod +x drone_script_test.py
sudo usermod -aG dialout Rpi
mavproxy.py --master=/dev/ttyACM0 --baudrate 57600
dmseg+
dmesg | grep tty
ls
gedit dronekit_testing.py 
gedit dronekit_testing01.py 
sudo reboot
sudo apt update
sudo apt upgrade -y
sudo raspi-config
python3 --version
sudo apt install python3-pip
pip3 --version
sudo apt install adafruit-circuitpython-bmp280
sudo pip3 install adafruit-circuitpython-bmp280
dmesg | grep tty
ls
python3 dronekit_testing01.py 
chmod +x drone_script_test.py
python3 drone_script_test.py
chmod +x drone_script_test.py
sudo reboot
cd ~
cd valt
touch bmp280_data.py
nano bmp280_data.py
python3 bmp280_data.py
gedit dronekit_testing01.py
python3 dronekit_testing01.py
ls
python3 dronekit_test.py
chmod +x dronekit_test.py
python3 dronekit_test.py
python3 dronekit_testing.py
gedit dronekit_testing.py
python3 dronekit_testing.py
gedit dronekit_testing.py
python3 drone_script_test.py
gedit drone_script_test.py 
python3 drone_script_test.py
python3 drone_test.py
chmod +x dronekit_test.py
gedit dronekit_test,py
gedit dronekit_test.py
python3 dronekit_test.py
gedit  dronekit_test.py
gedit dronekit_test.py
gedit dronekit_testing01.py
python3 dronekit_testing01.py
exit
sudo usermod -aG dialout Rpi
chmod +x drone_script_test.py
mavproxy.py --master=/dev/ttyACM0 --baudrate 57600
gedit dronekit_testing01.py
rm dronekit_test.py 
gedit dronekit_testing01.py
python3 dronekit_testing01.py
exit 
ls
cd bmp280
ls
cd src
ls
cd bmp280_ros
ls
cd scripts
ls
python3 bmp280_data.py
i2cdetect -y 1
gedit bmp280_data.py
python3 bmp280_data.py
pip3 install --upgrade adafruit-blinka adafruit-circuitpython-bmp280
sudo adduser Rpi i2c
sudo nano /boot/config.txt
sudo nano /etc/modules
sudo apt install -y i2c-tools
sudo reboot
sudo raspi-config
ls
cd bmp280
ls
cd src
ls
cd bmp280_ros
ls
cd scripts
ls
python3 bmp280_data.py
sudo python3 bmp280_data.py
gedit bmp280_data.py
nano bmp280_data.py
exit
ls
cd bmp280
ls
cd src
ls
cd bmp280_ros
ls
cd scripts
ls
python3 bmp280_data.py
ls
gedit bmp280_data.py
python3 bmp280_data.py
sudo python3 bmp280_data.py
lsmod | grep i2c
sudo modprobe i2c-dev
sudo modprobe 12c-bcm2835
echo "i2c-dev" | sudo tee -a /etc/modules
echo "i2c-bcm2835" | sudo tee -a /etc/modules
sudo i2cdetect -y 1
python3 bmp280_data.py
gedit bmp280_data.py
python3 bmp280_data.py
gedit bmp280_data.py
python3 bmp280_data.py
sudo python3 bmp280_data.py
sudo nano /boot/config.txt
sudo modprobe i2c-dev
sudo modprobe i2c-bcm2835
gedit bmp280_data.py
python3 bmp280_data.py
raspi-config
sudo raspi-config
sudo reboot
sudo i2cdetect -y 1
sudo pip install --upgrade pip
sudo pip3 install --upgrade pip3
sudo pip install RPI.BMP280
sudo pip install bmp280
--root-user-action
git clone https://github.com/bmp280-python
cd bmp280-python
cd examples
python3 temperature-and-pressure.py
sudo raspi-config
sudo reboot
cd bmp280-python
ls
cd examples
python temperature-and-pressure.py
python3 temperature-and-pressure.py
python temperature-and-pressure.py
nano temperature-and-pressure.py
gedit temperature-and-pressure.py
gedit bmp280_data.py
sudo apt update
sudo raspi-config
sudo apt upgrade -y
python3 --version
sudo apt install python3-pip
pip3 --version
sudo pip3 install adafruit-circuitpython-bmp280
cd ~
cd valt
touch bmp280_data.py
nano bmp280_data.py
python3 bmp280_data.py
nano bmp280_data.py
python3 bmp280_data.py
gedit bmp280_data.py
cd ..
gedit bmp280_data.py
sudo raspi-config
sudo ssh divas@192.168.21.87
ifconfig
iconfig
ifconfig
ls
exit
roscore
ls
cd bmp280
cd src/bmp280_ros
cd scripts
ls
python3 bmp280_data.py
ls
sudo chmod +x bmp280_data.
sudo chmod +x bmp280_data.py 
python3 bmp280_data.py
cd
cd bmp280
raspi-config
sudo raspi-config
cd src/bmp280_ros/scripts
ls
python3 bmp280_data.py
gedit bmp280_data.py
sudo reboot
exit
ls 
sudo mission planner 
sudo apt mission planner
cd 
cd Downloads/
cd MissionPlanner
ls
cd
python3 dronekit_testing01.py
ls
ls /dev/ttyACM0
exit
mavproxy --master=/dev/ttyACM0 --baudrate 57600
mavproxy --master=/dev/ttyACM0 --baudrate=57600
mavproxy.py --master=/dev/ttyACM0 --baudrate 57600
mavproxy.py --master=/dev/ttyACM0 --baudrate 57600
                                                                                                                                                                                                                                                                                                 python3 dronekit_testing01.py 
exit
cd bmp280
cd src
cd bmp280_ros
ls
gedit package.xml
cd scripts
cd
cd bmp280
catkin_make
source /home/Rpi/bmp280/devel/setup.bash
cd src/bmp280_ros/scripts
sudo python3 bmp280_data.py
gedit bmp280_data.py
cd
sudo apt-get install python3-smbus
sudo apt remove
sudo apt install ros-noetic-rospy
sudo apt autoremove
sudo apt update
sudo apt upgrade
cd bmp280/src/bmp280_ros/scripts
sudo python3 bmp280_data.py
cd
cd bmp280
sudo apt install -y lsb-release gnupg2
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo curl -sSL http://packages.ros.org/ros.key | sudo apt-key add -
source /opt/ros/noetic/setup.bash
sudo rosdep init
rosdep update
roscore
roscore
source /opt/ros/noetic/setup.bash
roscore
echo $ROS_PACKAGE_PATH
python3 -c "import rospy; print(rospy.__version__)"
sudo apt install python3-rospy
python3 -c "import sys; print(sys.path)"
cd bmp280/src/bmp280_ros/scripts
sudo python3 bmp280_data.py
gedit bmp280_data.py
cd bmp280/src/bmp280_ros/scripts
gedit bmp280_data.py
chmod +x bmp280_data.py
rosrun your_package_name bmp280_data.py
rosrun bmp280_ros bmp280_data.py
rostopic echo /bmp280/temperature
cd bmp280/src/bmp280_ros/scripts
rostopic echo /bmp280/temperature
cd 
roscore
cd bmp280/src/bmp280_ros/scripts
sudo python3 bmp280_data.py
cd
sudo i2cdetect -y 1
sudo apt update
sudo apt upgrade -y
sudo raspi-config
python3 --version
sudo apt install python3-pip
pip3 --version
sudo pip3 install adafruit-circuitpython-bmp280
                                                                                                                                                                                                                                                                     cd ws
ls
cd devel
ls
cd bmp280-python
ls
cd bmp280
ls
cd ..
cd src
ls
catkin_create_pkg DGT22 rospy sensor_msgs tf2 geometry_msgs std_msgs
ls
cd DGT22
ls
mkdir scripts
cd scripts
touch Dht22.py
ls
chmod +x Dht22.py
ls
code ..
gedit Dht22.py
nano Dht22.py
cd ..
cd
cd ws/src/DGT22/scripts
nano Dht22.py
python3 Dht22.py
nano Dht22.py
gedit Dht22.py
cd
ls
cd ws
ls
cd devel
ls
cd ..
ls
mkdir sensor_wks
cd sensor_wks/
cd src
mkdir src
catkin_make
cd
mkdir sens_wks
cd sens_wks
mkdir src
catkin_make
ls
cd src
ls
catkin_create_pkg temp  rospy sensor_msgs tf2 geometry_msgs std_msgs
cd temp
ls
mkdir scripts
cd scripts
touch dht22.py
gedit dht22.py
nano dht22.py
python3 dht22.py
nano dht22.py
python3 dht22.py
nano dht22.py
python3 dht22.py
nano dht22.py
python3 dht22.py
nano dht22.py
python3 dht22.py
nano dht22.py
python3 dht22.py
sudo pip3 install Adafruit_DHT
python3 dht22.py
cd temp
cd sens
cd wks
cd sens_wks
ls
cd src
ls
cd temp
ls
cd scripts
gedit dht22.py
python3 dht22.py
cd
cd sens_wks/src/temp/scripts
python3 dht22.py
roscore
cd sens_wks/src/temp/scripts
gedit dht22.py
python3 dht22.py
rosrun
roscore
cd sens_wks
cd src
cd temp
cd scripts 
python3 dht22.py
gedit dht22.py
python3 dht22.py
gedit dht22.py
python3 dht22.py
gedit sensor_data.csv
ls
cd mq135_ws/src
ls
cd mq135_sensor
cd scripts
ls
python3 mq135_data.py
sudo python3 mq135_data.py
ls
gedit sensor_data.csv
cd sens_wks
ls
cd src
cd sens_wks/src/temp/scripts
cd sens_wks
cd src
cd temp
cd scripts
python3 dht22.py
cd sensor_data.csv
gedit sensor_data.csv
cd mq135_ws
cd src
cd mq135_sensor
cd scripts sudo python3 mq135_data.py
sudo python3 mq135_data.py
ls
cd scripts
ls
sudo python3 mq135_data.py
gedit sensor_data.csv
ls
cd mq135_ws/src
ls
cd mq135_sensor
ls
cd scripts
ls
sudo python3 mq135_data.py
cd sens_wks/src/temp/scipts
cd sens_wks/src/temp/scripts
python3 dht22.py
cd sens_wks/src/temp/scripts
python3 dht22.py
cd sens_wks/src/temp/scripts
python3 dht22.py
cd sens_wks
ls
cd src
cd temp
cd scripts
ls
python3 dht22.py
ls
cd mq135_ws
ls
cd src
ls
cd mq135_sensor
ls
cd scripts
ls
python3 mq135_data.py
sudo python3 mq135_data.py
cd sens_wks
cd src
cd temp
ls
cd scripts
ks
ls
python3 dht22.py
pwd
chmod +x /home/Rpi/sens_wks/src/temp/scripts
python3 dht22.py
roscore
cd sens_wks
cd src/temp/scripts
python3 dht22.py
cd sens_wks
cd src/temp/scripts
python3 dht22.py
ls
cat sensor_data.csv
sudo python3 dht22.py
ls
cd sens_wks
source /home/Rpi/sens_wks/devel/setup.bash
cd src/temp/scripts
python3 dht22.py
gedit sensor_data.csv
ls
sudo raspi-config
cheese
sudo nano /boot/config.txt
ls /dev/video*
sudo apt update\
sudo apt update
sudo raspi-config
sudo apt update
sudo apt upgrade
sudo raspi-config
ls
sudo apt-get install -y git
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
exit
sudo nano /boot/firmware/config.txt
sudo reboot
sudo raspi-config
dmesg | grep -i video
ls /dev/video*
sudo nano /boot/firmware/config.txt
sudo reboot
ls
cd YDLidar-SDK
ls
cd sens_wks
cd src/temp/scripts
ls
python3 dht22.py
cd sens_wks/src/temp/scripts
sudo python3 dht22.py
gedit dht22.py
sudo python3 dht22.py
gedit sensor_dat.csv
gedit sensor_data.csv
sudo python3 dht22.py
ls
cd mq135_ws
ls
cd src
ls
cd mq135_sensor
ls
cd scripts
ls
sudo python3 mq135_data.py
cd sens_wks
cd src
cd temp
cd scripts
cd sens_wks/src/temp/scripts
python3 dht22.py
sudo python3 dht22.py
cd sens_wks/src/temp/scripts
gedit dht22.py
python3 dht22.py
cd sens_wks/src/temp/scripts
python3 dht22.py
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! autovideosink
cheese
lsmod | grep bcm2835
v4l2-ctl --list-devices
sudo modprobe bcm2835_v4l2
ls /dev/video*
sudo apt install libcamera-apps -y
libcamera-hello
sudo modprobe bcm2835_v4l2
echo "bcm2835_v4l2" | sudo tee -a /etc/modules
sudo apt install v4l-utils gstreamer1.0-tools gstreamer1.0-plugins-good -y
sudo nano /boot/firmware/config.txt
fg
sudo reboot
ls /dev/video*
sudo modprobe bcm2835_v4l2
ls /dev/video*
dmesg | grep -i camera
sudo modprobe unicam
uname -r
sudo apt update
sudo apt install raspi-config
sudo apt update
sudo apt install linux-image-$(uname -r) raspberrypi-kernel-headers
sudo apt update
sudo apt install linux-headers-$(uname -r)
sudo apt install bcm2835-v4l2
sudo apt update
sudo apt upgrade -y
sudo apt dist-upgrade -y
sudo reboot
sudo apt update
sudo apt install linux-headers-$(uname -r)
sudo apt install bcm2835-v4l2
sudo apt install linux-headers-$(uname -r)
lsblk
sudo dd if=/dev/mmcblk0 of=/dev/sda bs=4M status=progress
lsblk
sudo mkdir /media/pi/backup_sd
pwd
sudo mkdir -p /media/pi/backup_sd
sudo mount /dev/sda1 /media/pi/backup_sd
lsblk
sudo dd if=/dev/mmcblk0 bs=4M status=progress | gzip > /media/pi/backup_sd/raspi-backup.img.gz
sudo mv /media/pi /media/Rpi
sudo chown -R Rpi:Rpi /media/Rpi
sudo chown -R Rpi:Rpi /media/Rpi/backup_sd
lsblk -f
sudo mount -o umask=0000 /dev/sda1 /media/Rpi/backup_sd
sudo mkfs.ext4 /dev/sda1
sudo umount /dev/sda1
sudo mkfs.ext4 /dev/sda1
sudo mount /dev/sda1 /media/Rpi/backup_sd
sudo chown -R Rpi:Rpi /media/Rpi/backup_sd
lsblk -f
sudo dd if=/dev/mmcblk0 bs=4M status=progress | gzip > /media/Rpi/backup_sd/raspi-backup.img.gz
sudo sync
sudo umount /media/Rpi/backup_sd
sudo eject /dev/sda
ls /dev/video*
dmesg | grep -i camera
v4l2-ctl --list-devices
sudo modprobe bcm2835-v4l2
sudo apt update
sudo apt install v4l-utils
dmesg | grep -i camera
sudo apt install --reinstall linux-modules-extra-$(uname -r)
uname -r
sudo apt update
apt search linux-modules-extra
sudo apt install --install-recommends linux-raspi
sudo modprobe bcm2835-v4l2
cheese
dmesg | grep -i camera
sudo apt update
sudo apt upgrade
sudo apt install --install-recommends linux-generic-hwe-20.04
sudo reboot
uname -r
sudo modprobe bcm2835-v4l2
dmesg | grep -i camera
sudo nano /boot/firmware/config.txt
sudo reboot
v4l2-ctl --list-devices
libcamera-hello
dmesg | grep -i camera
lsmod | grep bcm2835
sudo apt update && sudo apt install -y v4l-utils libcamera-apps
dmesg | grep -i unicam
dmesg | grep -i ov5647
sudo modprobe bcm2835_v4l2
ls /dev/video*
sudo nano /boot/firmware/config.txt
sudo reboot
rosrun cv_camera cv_camera_node
sudo apt update
sudo apt install ros-noetic-cv-camera ros-noetic-image-transport
sudo apt install ros-noetic-image-transport ros-noetic-camera-info-manager
cd ~/catkin_ws/src
git clone https://github.com/OTL/cv_camera.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
rosrun cv_camera cv_camera_node
exit
ls
rm test.jpg
ls
rm test_image.jpg
xdg-open  display_screenshot_29.01.2025.png 
xdg-open display_screenshot_29.01.2025.png 
rm display_screenshot_29.01.2025.png 
ls
rm -r sensor_wks
ls
rm -r bmp280
ls
roscd usb_camera_pkg/
ls
cd launch
ls
gedit usb_camera_node.launch 
ls
roslaunch usb_camera_pkg usb_camera_node.launch
gedit usb_camera_node.launch 
roslaunch usb_camera_pkg usb_camera_node.launch
gedit usb_camera_node.launch 
roslaunch usb_camera_pkg usb_camera_node.launch
roslaunch usb_camera_pkg usb_camera_node.launch framerate:=30 width:=1280 height:=720 format:=mjpeg
roslaunch usb_camera_pkg usb_camera_node.launch framerate:=30 width:=640 height:=480 format:=mjpeg
gedit usb_camera_node.launch 
rosrun usb_camera_pkg usb_camera_node _auto_exposure:=true _auto_white_balance:=true _image_width:=640 _image_height:=480 _framerate:=30 _pixel_format:=mjpeg
roslaunch usb_camera_pkg usb_camera_node.launch _auto_exposure:=true _auto_white_balance:=true _image_width:=640 _image_height:=480 _framerate:=30 _pixel_format:=mjpeg
gedit usb_camera_node.launch 
lsof /dev/video0
gedit usb_camera_node.launch 
lsof /dev/video0
gedit usb_camera_node.launch 
ls
cd camera_node/
cd src
ls
cd cone_detection/
ls
cd scripts/
ls
python3 cone_detection.py 
roscore
roscd camera_n
roscd camera_node
roscd cone_detection/
ls
cd scripts/
ls
python3 cone_detection.py 
pwd
ls
ls
cd usb_camera/
roscd usb_cam
ls
cd launch
ls
gedit usb_cam-test.launch 
cd
ls
gedit dronekit_testing.py
gedit dronekit_testing01.py
ls
gedit dronekit_script_test.py
gedit drone_script_test.py
ls
cs ws
cd ws
ls
cd ws
ls
cd src
ls
cd
cd sens_wks/
ls
cs src
cd src
ls
cd temp/
ls
cd scripts/
ls
cdcd
cd
ls
gedit dronekit_main.py
xdg-open image.png
cd images
xdg-open image_0.png
xdg-open image_0.jpg
ls -l images/
cd
ls -l images/
rostopic list
rostopic echo /camera/image_raw
rostopic hz /camera/image_raw
cd images
xdg-open image_0.jpg
ls
cd
# Create the workspace directory
mkdir -p ~/catkin_ws/src
# Navigate to the workspace directory
cd ~/catkin_ws/
# Initialize the workspace
catkin_make
# Source the workspace
source devel/setup.bash
sudo apt update
sudo apt install ros-noetic-v4l2-camera
cd
sudo apt install ros-noetic-usb-cam
cd ~/catkin_ws/src
catkin_create_pkg my_camera_package std_msgs rospy sensor_msgs
cd ~/catkin_ws/src/my_camera_package
mkdir launch
nano launch/camera.launch
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch my_camera_package camera.launch
cd
cdimages
cd images
xdg-open image_0.jpg
cd
nano launch/camera.launch
cd launch
cd
cd catkin_ws/
cd src/
cd my_camera_package/
cd launch
nano launch/camera.launch
cd ..
mkdir launch
nano launch/camera.launch
cd
xdg-open image_0.jpg
cd images
xdg-open image_0.jpg
ls
cd
rostopic echo /camera/image_raw
roslaunch usb_cam usb_cam_node.launch
rospack list | grep usb_cam
roslaunch usb_cam usb_cam_node.launch
rostopic list
rostopic echo /camera/image_raw
ls /dev/video*
roslaunch usb_cam usb_cam_node.launch
roscd usb_cam
cd
cd launch
ls
cd
roscd usb_cam
cd launch
ls
cd
cd ~/catkin_ws/src  # Navigate to your ROS workspace source directory
catkin_create_pkg my_camera_package std_msgs rospy sensor_msgs cv_bridge usb_cam
cd ~/catkin_ws/src/my_camera_package
cd ~/catkin_ws
catkin_make
source devel/setup.bash
mkdir -p launch
cd launch
nano usb_cam_node.launch
cd ~/catkin_ws
source devel/setup.bash
roslaunch my_camera_package usb_cam_node.launch
cd
roslaunch my_camera_package usb_cam_node.launch
roscd my_camera_package/
ls
launch
cd launch 
ls
roslaunch camera.launch 
cd
cd images/
xdg-open image_0.jpg
ls
roslaunch v4l2_camera v4l2_camera.launch
rostopic list
sudo apt update
sudo apt upgrade
sudo apt install ros-$ROS_noetic-v4l2-camera
sudo apt install ros-$ROS_NOETIC-v4l2-camera
ECHO ROS_NOETIC
echo ROS_NOETIC
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/v4l2_camera.git
cd
sudo apt install ros-noetic-v4l2-camera
sudo apt install ros-noetic-usb-cam
sudo apt install ros-noetic-raspicam-node
ls /dev/video*
v4l2-ctl --list-formats-ext
roslaunch v4l2_camera v4l2_camera.launch
roslaunch usb_cam usb_cam-test.launch
rostopic E: Unable to locate package ros--v4l2-camera
roslaunch usb_cam usb_cam-test.launch
roslaunch usb_camera_pkg usb_camera_node.launch
cd usb_camera
cd src/
cdd usb_camera_pkg/
cd usb_camera_pkg/
cd launch/
roslaunch usb_camera_pkg usb_camera_node.launch
ls
cd
roslaunch usb_cam usb_cam-test.launch
cd /catkin_ws/src/my_camera_package/launch
cd ~/catkin_ws/src/my_camera_package/launch
gedit camera.launch 
nano camera.launch 
cd
roslaunch usb_cam usb_cam-test.launch
ls /dev/video*
v4l2-ctl --list-devices
rostopic list
roslaunch usb_cam usb_cam-test.launch
roscore
python3 panorama_ros.py
python3 panorama.py
ls
gedit panorama_ros.py
ls
python3 panorama.py
python3 panorama_ros.py
sdg -open image_0.jpg
stg -open image_0.jpg
xtg -open image_0.jpg
xdg -open image_0.jpg
ls
cd Pictures
ls
xdg-open image.png
cd
xdg-open image.png
xdg-open panorama_ros.png
xdg-open panorama.png
xdg-open image0.png
xdg-open image_0.png
ls
cd images
xdg-open image_0.png
ls
xdg-open image_0.jpg
cd
python3 panorama_ros.py
rostopic list
python3 panorama_ros.py
rostopic list
rostopic echo /usb_cam/image_raw
rosrun image_view image_view image:=/usb_cam/image_raw
python3 panorama_ros.py
python3 23.py
python3 sample.py
nano sample.py
python3 sample.py
nano sample.py
python3 sample.py
nano sample.py
python3 sample.py
nano sample.py
python3 sample.py
nano sample.py
python3 sample.py
nano sample.py
python3 sample.py
nano sample.py
python3 sample.py
nano sample.py
python3 sample.py
nano sample.py
python3 sample.py
nano sample.py
python3 sample.py
nano sample.py
python3 sample.py
nano sample.py
python3 sample.py
nano sample.py
python3 sample.py
nano sample.py
python3 sample.py
nano sample.py
python3 sample.py
nano sample.py
python3 sample.py
nano sample.py
python3 sample.py
nano sample.py
python3 sample.py
nano sample.py
python3 sample.py
nano sample.py
python3 sample.py
nano sample.py
python3 sample.py
nano sample.py
y
python3 sample.py
nano sample.py
code sample.py 
code  sample.py 
code sample.py 
sample.py code
ls
gedit dronekit_main.py
rosrun cone_detection cone_detection.py 
v4l2-ctl -d /dev/video0 --list-ctrls | grep focus
gedit focus_detect.py
python3 focus_detect.py 
gedit focus_detect.py
python3 focus_detect.py 
rosrun cone_detection cone_detection.py 
gedit toggle.py
chmod +x toggle.py
chmod +x dronekit_main_1.py
ls
exit
roscore
exit
ls
gedit dronekit_main_1.py
exit
python3 toggle.py 
gedit toggle.py
python3 toggle.py 
exit
roscore
exit
roscore
exit
roslaunch usb_camera_pkg usb_camera_node.launch 
exit
rosrun cone_detection cone_detection.py 
exit
cd sens_wks
cd src
cd temp
ls
cd scripts
ls
python3 dht22.py
sudo python3 dht22.py
ls
ls -l
ls
cd sens_wks/
la
cd src
ls
cd temp
ls
cd scrpit
cd scripts
ls
python3 dht22.py
ls -l
chmod +x dht22.py 
ls 
python3 dht22.py
cd sens_wks
cd src
cd temp
cd scripts
python3 dht22.py
cd sens_wks/src/temp/scripts
python3 dht22.py
la
cd mq135_ws
ls
cd src
ls
cd mq135_sensor
ls
cd scripts
ls
sudo python3 mq135_data.py
ls
cd mq135_ws
cd src
ls
cd mq135_sensor
ls
cd scripts
ls
sudo python3 mq135_data.py
cd sens_
cd sens_wks
cd src
cd temp
cd scripts
ls
python3 dht22.py
iwconfig
sudo apt update
sudo apt install dkms git
sudo apt install build-essential libelf-dev linux-headers-$(Rpi -r)
git clone https://github.com/aircrack-ng/rtl8812au.git
cd rtl88*
sudo make dkms_install
exit
python3 
gedit sample.py
ls
python3 final.py 
exit
ls
gedit dronekit_testing01.py 
gedit verticalpano.py 
exit 
cd sens_wks/src/temp/src
cd ..
cd scripts
python3 dht22.py
sudo python3 dht22.py
cd sens_wks/src/temp/src
cd ..
cd scripts
python3 dht22.py
cd sens_wks/src/temp/src
cd ..
cd scripts
python3 dht22.py
cd sens_wks
cd src/temp/scripts
python3 dht22.py
cd 
cd mq135_ws/src/mq135_sensor/scripts
sudo python3 mq135_data.py
import rospy
gedit dronekit_main_1.py
gedit dronekit_main.py
exit 
ls
nano dronekit_testing01.py 
nano final.py 
nano dronkit2.py 
gedit dronekit_testing01.py 
exit
ls
gedit dronekit_main.py
cd catkin_ws
cd src
cd cd ~/catkin_ws/src
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
cd ~/catkin_ws/src
catkin_create_pkg drone_pano rospy std_msgs sensor_msgs cv_bridge image_transport
cd ~/catkin_ws
catkin_make
source devel/setup.bash
rospack list | grep drone_pano
cd ~/catkin_ws/src/drone_pano
mkdir -p scripts launch
cd ~/catkin_ws/src/drone_pano/scripts
touch panorama_zigzag.py
chmod +x panorama_zigzag.py
nano panorama_zigzag.py
roslaunch drone_pano drone_panorama.launch
roslaunch drone_pano zigzag_panorama.launch
roslaunch drone_pano panorama_zigzag.launch
cd ~/catkin_ws/src
catkin_create_pkg drone_pano rospy std_msgs sensor_msgs geometry_msgs mavros_msgs cv_bridge
cd ~/catkin_ws
catkin_make
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
roslaunch drone_pano panorama_zigzag.launch
cd
roslaunch drone_pano panorama_zigzag.launch
rospack list | grep drone_pano
ls ~/catkin_ws/src/drone_pano/launch/
touch ~/catkin_ws/src/drone_pano/launch/panorama_zigzag.launch
nano ~/catkin_ws/src/drone_pano/launch/panorama_zigzag.launch
roslaunch drone_pano panorama_zigzag.launch
nano ~/catkin_ws/src/drone_pano/launch/panorama_zigzag.launch
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch drone_pano panorama_zigzag.launch
cd src/drone_pano/scripts
roslaunch drone_pano panorama_zigzag.launch
ls
cd mq135_wks
cd mq135_ws
cd src/mq135_sensor
cd scripts
python3 mq135_data.py
gedit mq135_data.py
python3 mq135_data.py
gedit mq135_data.py
python3 mq135_data.py
gedit mq135_data.py
python3 mq135_data.py
sudo nano /boot/config.txt
python3 mq135_data.py
exit
python3 toggle.py 
exit
python3 toggle.py 
exit
python3 fullpano.py
ls
cd mq135_ws
cd src
cd mq135_sensor
cd scripts
ls
sudo python3 mq135_data.py
gedit mq135_data.py
sudo python3 mq135_data.py
cd sens_wks
cd src/temp/scripts
python3 dht22.py
cd sens_wks
cd src/temp/scripts
python3 dht22.py
cd sens_wks/src/temp/scripts
python3 dht22.py
cd mq135_ws
ls
cd src
ls
cd mq135_sensor
ls
cd scripts
ls
sudo python3 mq135_data.py
cd sens_wks
source /home/Rpi/sens_wks/devel/setup.bash
catkin_make
source /home/Rpi/sens_wks/devel/setup.bash
cd src/temp/scripts
sudo python3 dht22.py
cd sens_wks/src/temp/scripts
python3 dht22.py
cd mq135_ws
ls
cd src
ls
cd mq135_sensor
ls
cd scripts
ls
cd mq135_data.py
sudo python3 mq135_data.py
ls
exit 
gedit dronekit_testing01.py 
exit 
ls
gedit verticalpano.py 
ls
gedit dronekit_main.py 
exit 
roscore
rosrun cone_detection cone_detection.py 
exit
roslaunch usb_cam usb_cam-test.launch
scp best.pt rpi@192.168.1.100:/home/rpi/
scp best(13).pt rpi@192.168.1.100:/home/rpi/
hostname -I
scp best(13).pt rpi@192.168.143.75:/home/rpi/
scp best\(13\).pt rpi@192.168.143.75:/home/rpi/
scp best(13).pt Rpi@192.168.143.75:/home/rpi/
scp best\(13\).pt Rpi@192.168.143.75:/home/rpi/
scp best\(13\).pt Rpi@192.168.143.75:/home/Rpi/
scp best\(13\).pt rpi@192.168.143.75:/home/rpi/
scp best\(13\).pt rpi@192.168.143.75:/home/Rpi/
123456789
scp best.pt rpi@Rpi:/home/rpi/
exit
LS
ls
ls
mkdir cone_images
ls
cd cone_images/
roscd cone_detection/
cd scripts
ls
gedit cone_detection1.py
cd ~/camera_node/src/cone_detection/scripts
ls
gedit cone_detection1.py
roscore
roscd cone_detection/
cd scripts
ls
gedit cone_detection1.py
roscore
gedit dronekit_final.py 
roscd cone_detection/scripts
ls
gedit cone_detection1.py
roscd cone_detection/scripts
gedit cone_detection1.py
roscd cone_detection/scripts/
gedit cone_detection1.py
exit
roscore
roscd usb_camera_pkg/ 
cd scripts/
ls
gedit capture_image.py 
cd
gedit dronekit_final.py 
exit 
ls
gedit dronekit_main_1.py 
gedit dronekit_main_2.py 
gedit dronekit_testing.py 
gedit dronekit_testing01.py 
gedit dronekit_final.py
exit
gedit dht22.py
exit
ls
rm panorama.jpg 
ls
rm panorama)1.jpg 
rm panorama_0.jpg 
rm panorama_1.jpg 
rm panorama_2.jpg 
ls
ls catkin_ws/
ls
cd catkin_ws/
ls
cd launch/
ls
cd
gedit dronekit_final.py 
gedit dronekit_final.py 

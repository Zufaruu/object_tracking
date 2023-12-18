# Object Tracking System
Part of Intelligent Surveillance System Drone Project MSIB V PT Len Industri (Persero)

# Setup
## Devices
- Jetson Nano SUB Board
- Jetson Nano Power Adapter 5V 5A
- Monitor Display
- HDMI-to-HDMI cable
- USB-to-TTL cable or FTDI cable
- EH10 Camera 
- EH10 Camera Power Cable
- Mouse and Keyboard
- Micro HDMI-to-HDMI cable
- HDMI Video Capture

## Assembly
- Jetson Nano Power Adapter 5V 5A -> Jetson Nano DC Barrel Jack Power Input
- Jetson Nano HDMI Output Port -> HDMI-to-HDMI cable -> Monitor Display HDMI Input Port
- Mouse and Keyboard -> Jetson Nano USB Ports
- EH10 Micro HDMI Output Port -> Micro HDMI-to-HDMI cable -> HDMI Video Capture -> Jetson Nano USB Ports
- EH10 (Tx Rx GND) pins -> USB-to-TTL (Rx Tx GND) cable -> Jetson Nano USB Ports

## Jetson Nano Setup
1. Follow tutorial no 3.2 (Programming EMMC System) to install Jetson Nano OS using SDKManager from this link http://www.yahboom.net/study/jetson-nano
2. After booting, open a terminal and run this command to install the necessary packages <br>
   `sudo apt-get update` <br>
   `sudo apt-get upgrade` <br>
   `sudo apt-get install python3-pip` <br>
   `python3 -m pip install --upgrade pip` <br>
   `python3 -m pip install opencv-contrib-python==3.14.18.65` <br>
3. Clone this repo and run `python3 tracking.py`
   

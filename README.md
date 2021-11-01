# ROS driver for the ICM-20948 
[![](https://img.shields.io/badge/ROS-Noetic-brightgreen.svg)](https://github.com/Alpaca-zip/PupBot)
## 1 Installation
### 1.1 Install ROS package on PC 
```
$ cd catkin_ws/src 

$ git clone https://github.com/Alpaca-zip/icm_20948.git 

$ cd .. && catkin_make
```
### 1.2 Download Arduino IDE on PC 
Get the latest version from the download page.

https://www.arduino.cc/en/software
## 2 For Seeeduino Xiao 
### 2.1 ICM-20948 interfacing with Seeeduino Xiao 
<img src="https://github.com/Alpaca-zip/icm_20948/blob/main/ICM-20948.png" width="320px">

### 2.2 Preferences 
After Arduino IDE is run, click File -> Preferences in the top menu of the IDE. When the Preferences window appears, copy and paste following link to the Additional Boards Manager URLs textbox. 

https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json 

### 2.3 Install the board package via Boards Manager 
Click Tools -> Board -> Boards Manager..., print keyword "Seeeduino XIAO" in the searching blank. Here comes the "Seeed SAMD Boards". Install it. 

### 2.4 Select your board and port 
Click Tools -> Board, find "Seeeduino XIAO M0" and select it. Now you have already set up the board of Seeeduino XIAO for Arduino IDE. Then, Select the serial device of the Arduino board from the Tools | Serial Port menu. This is likely to be COM3 or higher.  

### 2.5 Download the Arduino library for ICM-20948 and ROS 
Click Sketch -> Include Library -> Manage Libraries... 

### 2.5.1 ICM-20948 library 
Print keyword " ICM 20948" in the searching blank. Here comes the "SparkFun 9DoF IMU Breakout - ICM-20948". Install it. 

***Important note:**

**By default the DMP functionality is disabled in the library as the DMP firmware takes up 14301 Bytes of program memory.**

**To use the DMP, you will need to:**

**1, Edit ICM_20948_C.h**

**2, Uncomment line 29: #define ICM_20948_USE_DMP**

**3, Save changes**

**If you are using Windows, you can find ICM_20948_C.h in:**

**Documents\Arduino\libraries\SparkFun_ICM-20948_ArduinoLibrary\src\util**

### 2.5.2 ROS library 
Print keyword " Rosserial " in the searching blank. Here comes the " Rosserial Arduino Library". Install it. 

### 2.6 Upload the program 
Open the “include/ICM_20948_ROS_Example/ICM_20948_ROS_Example.ino”. Then, simply click the "Upload" button in the environment. Wait a few seconds and if the upload is successful, the message "Done uploading." will appear in the status bar. 

## 3 Usage 
### Launch icm_20948_example node 
To run the node with a different port, for example on /dev/ttyACM0, you must specify the “PORT_NAME” parameters on the command line: 
```
$ roslaunch icm_20948 icm_20948_example.launch PORT_NAME:=/dev/ttyACM0
```
If the node is successfully launched, the terminal will print below messages and RViz will be opened. 
<img src="https://github.com/Alpaca-zip/icm_20948/blob/main/Terminal.png" width="600px">

<img src="https://github.com/Alpaca-zip/icm_20948/blob/main/RViz.png" width="600px">

If you want to see other IMU data, you will need to: 

1, Edit icm_20948_example.cpp 

2, Uncomment line 15~17 

3, Save changes 

4, catkin_make 

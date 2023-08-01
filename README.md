# ICM_20948 v2.0 [![ROS-noetic Build Check](https://github.com/Alpaca-zip/icm_20948/actions/workflows/ros1-build-check-bot.yml/badge.svg?event=pull_request)](https://github.com/Alpaca-zip/icm_20948/actions/workflows/ros1-build-check-bot.yml)

ROS package for the ICM-20948 with Seeeduino XIAO.  
Older version: [old-devel](https://github.com/Alpaca-zip/icm_20948/tree/old-devel)

<img src="https://user-images.githubusercontent.com/84959376/218235948-b36ffe70-4e4e-4186-acaf-ef2bd89b3470.png" width="320px"> 　　　　<img src="https://github.com/Alpaca-zip/icm_20948/assets/84959376/807b0fb9-970a-4a2f-97f3-b76d624296f2" width="400px">

## 1 Installation
### 1.1 Install ROS package
```
$ cd ~/catkin_ws/src
$ git clone -b noetic-devel https://github.com/Alpaca-zip/icm_20948.git 
$ cd ~/catkin_ws
$ rosdep install -r -y -i --from-paths .
$ catkin build
```

### 1.2 Download Arduino IDE
Get the latest version from the [download page](https://www.arduino.cc/en/software).

## 2 For Seeeduino XIAO
### 2.1 Interfacing with Seeeduino XIAO
|  ICM-20948  |  Seeeduino XIAO  |
| :----------------: | :------------: |
| GND | GND |
| 3V3 | VIN |
| DA  | SDA |
| CL  | SCL |

### 2.2 Preferences
After Arduino IDE is run, click `File -> Preferences` in the top menu of the IDE. When the Preferences window appears, copy and paste following link to the Additional Boards Manager URLs textbox.

`https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json`

### 2.3 Install the board package via Boards Manager
Click `Tools -> Board -> Boards Manager`..., print keyword "Seeeduino XIAO" in the searching blank. Here comes the "Seeed SAMD Boards". Install it.

### 2.4 Select your board and port
Click `Tools -> Board`, find "Seeeduino XIAO M0" and select it. Now you have already set up the board of Seeeduino XIAO for Arduino IDE. Then, Select the serial device of the Arduino board from `Tools -> Port` menu. This is likely to be COM3 or higher.

### 2.5 Download the Arduino library for ICM-20948
Click `Sketch -> Include Library -> Manage Libraries`..., print keyword " ICM 20948" in the searching blank. Here comes the "SparkFun 9DoF IMU Breakout - ICM-20948". Install it.

> ***Important note:**  
By default the DMP functionality is disabled in the library as the DMP firmware takes up 14301 Bytes of program memory.  
To use the DMP, you will need to:  
> 1. Edit ICM_20948_C.h  
> 2. Uncomment line 29: #define ICM_20948_USE_DMP  
> 3. Save changes
> 
> If you are using Windows, you can find ICM_20948_C.h in:  
Documents\Arduino\libraries\SparkFun_ICM-20948_ArduinoLibrary\src\util

### 2.6 Upload the sketch
Open the “firmware/firmware.ino”. Then, simply click the `Upload` button in the environment. Wait a few seconds and if the upload is successful, the message "Done uploading." will appear in the status bar.

## 3 Usage
To run the node with a different port, for example on /dev/ttyACM0, you must specify the “port” parameters on the command line:

```
$ roslaunch icm_20948 run.launch port:=/dev/ttyACM0 debug:=true
```

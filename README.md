The purpose of this github page is to document my progress throughout the project. In the future I also want this github page to serve as a guide for other people to build their own robot, so some of the text is written in this 'how to do stuff' way.

# Table of Contents
* [Introduction](#introduction)
* [Software used](#software-used)
* [Hardware used](#hardware-used)
  * [Parts](#parts)
* [Robot design](#robot-design)
* [Setting up ROS melodic](#setting-up-ros-melodic)
  * [ROS on RPI](#ros-on-rpi)
  * [ROS on laptop](#ros-on-laptop)
* [Installing RTABMAP](#installing-rtabmap)
  * [RTABMAP on RPI](#rtabmap-on-rpi)
    * [freenect](#freenect)
    * [freenect_stack](#freenect_stack)
    * [RTABMAP_ros](#rtabmap_ros) 
  * [RTABMAP on laptop](#rtabmap-on-laptop)
  * [Testing RTABMAP](#testing-rtabmap)
* [Communication between RPI and Arduino](#communication-between-rpi-and-arduino)
* [Controlling the motors](#controlling-the-motors)
  * [Arduino code for controlling the motors](#arduino-code-for-controlling-the-motors)
  * [Testing Arduino code](#testing-arduino-code)

# Introduction
This project came about because of the COVID-19 pandemic. In normal times I would have had the opportunity to work with the turtlebot3 for one of the courses I am enrolled in.

Since we were not allowed to have hands-on exercise sessions and sending every student a turtlebot3 was not possible, the exercise sessions mainly consisted of completing partially written code in Python. I should note however that the goal of the exercise session was not to learn how to write code for a robot, but to grasps the concepts from the lectures (e.g. the kallman filter and different path planning algorithms). 

While the exercise sessions succeeded in this area, I was still itching to learn how to write code for a robot and to see this code in action. Throughout the semester I started my research on what was needed to make a turtlebot3-like robot and devised a sort of action plan. After my exams I finally started this project.

Tasks:
- [x] Setting up ROS on rpi and laptop.
- [x] Installing RTABMAP ros.
- [x] Testing RTABMAP ros. 
- [x] Writing code for controlling the motors.
- [x] Testing the code for controlling the motors.
- [ ] Writing code to read encoder data.
- [ ] Testing reading encoder data code.
- [ ] Writing code to publish the encoder data in a `nav_msgs/Odometry` message format.
- [ ] Testing `nav_msgs/Odometry` message code
- [ ] Configure RTABMAP ROS
- [ ] Testrun

# Software used

 - [Ubuntu 18.04.5 LTS](https://releases.ubuntu.com/18.04/) for the laptop
 - [Ubuntu 18.04.5 LTS server](https://cdimage.ubuntu.com/releases/18.04/release/) (from a preinstalled server image) for the Raspberry Pi
 - [ROS melodic](http://wiki.ros.org/melodic)
 - [Arduino IDE](https://www.arduino.cc/en/software)

# Hardware used

 - A laptop
 - Raspberry Pi 3B+ (from now on referred to as RPI)
 - Arduino Uno Rev3
 - Microsoft xbox 360 Kinect (model 1414) + power adapter
 - Cytron MDD10A DC motor driver
 - ~~Pololu 380:1 micro metal gearmotor HPCB 12V with extended motor shaft~~ Wrong motor arrived so for now using Pololu 30:1 micro metal gearmotor 12V without extended shaft
 - Pololu 12CPR magnetic encoder kit
 - Pololu 1/2" metal ball caster
 - Xiaomi 10000 mAh powerbank
 - 12 V battery holder

## Parts

 - Various nuts and bolts
 - Plastic 6mm threaded rod
 - Plasting M6 nuts
 - 3mm and 6mm mdf plates

# Robot design
# Setting up ROS melodic
## ROS on RPI
Installing ROS melodic is pretty easy if you follow the [wiki](http://wiki.ros.org/melodic/Installation/Ubuntu). The first step, configuring the Ubuntu repositories, can be done on a headless setup by editing the sources.list file. See the [Ubuntu documentation](https://help.ubuntu.com/community/Repositories/Ubuntu) for more information. Also, installing the ROS-Base version will suffice.

After installing ros we'll need to create a ROS workspace. Again, the ROS wiki has good [guide](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) on how to do this. Since we're not going to create other workspaces we can add this workspace to bashrc with `echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`.
## ROS on laptop
The installation proces is almost exactly the same as the installation on the RPI. The differences with the RPI installation are that you need to install the Dekstop-Full version and that you don't need to create a workspace.
# Installing RTABMAP
RTABMAP will be used for SLAM.
## RTABMAP on RPI
### freenect
Freenect is the driver for the Kinect. At the time of writing the apt version of freenect is outdated, so we'll be building freenect from source. 
```bash
sudo apt-get install git-core cmake freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libusb-1.0-0-dev

git clone git://github.com/OpenKinect/libfreenect.git

cd libfreenect

mkdir build

cd build

cmake ..

make

sudo make install

sudo ldconfig /usr/local/lib64/
```
After building freenect from source we need to create a permission file so we can use the kinect as a normal user. This is done with `sudo nano /etc/udev/rules.d/51-kinect.rules` and entering the following in the empty file:
```
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666"
# ATTR{product}=="Xbox NUI Audio"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666"
# ATTR{product}=="Xbox NUI Camera"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666"
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c2", MODE="0666"
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02be", MODE="0666"
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02bf", MODE="0666"
```
We can test if freenect is working by entering `lsusb` in a new terminal window when the Kinect is connected to the RPI. You should see the Kinect listed.

*source: [freenect wiki](https://openkinect.org/wiki/Getting_Started#Ubuntu_Manual_Install)*

### freenect_stack
Thankfully, installing freenect_stack only requires one command: 
```bash
sudo apt install ros-melodic-freenect-stack
```
*source: [freenect_stack on the ROS wiki](http://wiki.ros.org/freenect_stack)*

### RTABMAP_ros
And so does RTAMAP_ros:
```bash
sudo apt install ros-melodic-rtabmap-ros
```
*source: [RTABMAP_ros on the ROS wiki](http://wiki.ros.org/rtabmap_ros)*

## RTABMAP on laptop
RTABMAP on our laptop will only be used to display stuff with rviz of rtabmap's version of rviz. Install with:
```bash
sudo apt install ros-melodic-rtabmap-ros
```
## Testing RTABMAP
Follow the [remote mapping tutorial](http://wiki.ros.org/rtabmap_ros/Tutorials/RemoteMapping).
# Communication between RPI and Arduino
We'll use an Arduino to control the motors of our robot. The communication between the RPI and the Arduino will be done with [rosserial](http://wiki.ros.org/rosserial_arduino). Install rosserial on the RPI with:
```bash
sudo apt install ros-melodic-rosserial-arduino
sudo apt install ros-melodic-rosserial
```
To install the library for the Arduino IDE we have two options:

 1. Use the library manager in the Arduino IDE. Search for rosserial and install version 0.7.9. (at the time of writing version 0.9.1 will give an error when compiling)
 2. Let rosserial make the library with `rosrun rosserial_arduino make_libraries.py`. Follow the instructions on the [rosserial ros wiki](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup). 

We can test if rosserial is working by using the [Hello World example](http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World). 

  * **NB:** To run rosserial we use the command `rosrun rosserial_python serial_node.py /dev/ttyACM0`. The `/dev/ttyACM0` part may be different on your setup. Run `dmesg` in a terminal window while the arduino is connected to the RPI.

*source: [rosserial on the ROS wiki](http://wiki.ros.org/rosserial_arduino)*
# Controlling the motors
The ros topic `/cmd_vel` will receive commands from RTABMAP in the form of `geometry_msgs/Twist` messages. Looking at the [API](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Twist.html) we can see that this message consists of two vectors; one for linear velocities and one for angular velocities.
Since our robot can only move in a 2D plane we're only interested in the x and y component of the linear velocity vector and in the z component of the angular velocity vector.
## Arduino code for controlling the motors
See the full code in PATH_TO_CODE. The gist of it is that we subscribe to the `/cmd_vel` topic, convert the `geometry_msgs/Twist` message into a motor speed and then convert this motor speed into a command for our motor driver.
## Testing Arduino code
We'll use [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard), which converts key presses into `geometry_msgs/Twist` messages which then get published to the `/cmd_vel` topic.
Install with:
```bash
sudo apt-get install ros-melodic-teleop-twist-keyboard
```
We could test our code by running the following command

 1. Run `roscore`
 2. Run `rosrun rosserial_python serial_node.py /dev/ttyACM0` in a new terminal window.
 3. Run `rosrun teleop_twist_keyboard teleop_twist_keyboard.py` in a new terminal window.

But to make our lives easier we're going to create a launch file. This way we'll only need to enter one command.

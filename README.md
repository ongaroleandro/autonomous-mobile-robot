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
  *  [Launch file](#launch-file)
* [Navigation stack](#navigation-stack)
  * [Calculating the pose from odometry](#calculating-the-pose-from-odometry)  
    * [First iteration of python code](#first-iteration-of-python-code)  
    * [Second iteration of python code](#second-iteration-of-python-code) 

# Introduction
This project came about because of the COVID-19 pandemic. In normal times I would have had the opportunity to work with the turtlebot3 for one of the courses I am enrolled in.

Since we were not allowed to have hands-on exercise sessions and sending every student a turtlebot3 was not possible, the exercise sessions mainly consisted of completing partially written code in Python. I should note however that the goal of the exercise session was not to learn how to write code for a robot, but to have a firmer grasp on the concepts from the lectures (e.g. the kallman filter and different path planning algorithms). 

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
 - Pololu 380:1 micro metal gearmotor HPCB 12V with extended motor shaft
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

## Launch file
The ros wiki has a [great](http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch#Using_roslaunch) tutorial on making a launch file and luckily it is pretty straightforward. Our launch file will have to launch rosserial and teleop_twist_keyboard. The launch file is located at `ROS/src/testing/src/`.

# Navigation stack
The navigation stack is essential for our robot since this is what makes our robot mobile. The navigation stack is well documented on the ROS wiki, and there's even a [tutorial](http://wiki.ros.org/navigation/Tutorials/RobotSetup) on how to set up a navigation stack.

Here there is a handy image to give us an idea of the different components of the navigation stack. Note the blue boxes, these are the things we'll need to provide to the navigation stack. The bottom box, base controller, should seem familiar because we already made this part in [Controlling the motors](#controlling-the-motors). 

Our first order of business is to create an odometry source node which publishes an `odom`topic. This topic contains a `nav_msgs/Odometry`. 

Looking at the [API](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) of this message, we can see that it contains the pose and twist of our robot. The API also states that the pose is with respect to the frame specified in the header, while the twist is with repect to the child frame. According to [REP105](https://www.ros.org/reps/rep-0105.html), the header frame will be the odom frame and the child frame will be the base_link frame. The base_link moves with the robot while the odom frame is a fixed frame. At this point in time we'll assume the origin of the map frame and the origin of the  odom frame coincide and that our robot always starts at the origin of the odom frame.

With those assumptions we can determine the pose from our odometry, i.e. the two wheel encoders. 

 
## Calculating the pose from odometry

The calculation of the pose will be done by a separtate subscriber node, which subscribes to our arduino data publisher.

The [pose](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html) contains our position and orientation in free space. Our robot is restricted to a single plane, the XY plane, so the pose of our robot is the x- and y-postion and the angle with respect to the x-axis.

The kinematics of a 2 wheel differential drive robot are well known. [This](https://www.hmc.edu/lair/ARW/ARW-Lecture01-Odometry.pdf) lecture by Chris Clark gives us the equations we need and also explains the derivation of them step by step. 
The equations are:

![pose equations](media/pose%20equations.png)

### First iteration of python code
*The idea for this code was to calculate the pose of our robot but I also wanted to keep a list with all locations our robot has been. This list will then be written to a csv file when this node is closed so I could look at the data afterwards.*

The equations translated into Python code (the full code can be found at `proof of concept phase/`):

```python
x_y_theta_t = [[0, 0, 0, 0]]
r_wheel = 0.025
wheel_sep = 0.210


def calc_pos(msg):
    DIR_L = msg.data[0]
    wl = msg.data[1]
    DIR_R = msg.data[2]
    wr = msg.data[3]
    t_ard = msg.data[4]
   
    delta_theta = (-wl * r_wheel * (t_ard - x_y_theta_t[-1][3]) + wr * r_wheel * (t_ard - x_y_theta_t[-1][3])) / wheel_sep
    delta_s = (wl * r_wheel * (t_ard - x_y_theta_t[-1][3]) + wr * r_wheel * (t_ard - x_y_theta_t[-1][3])) * 0.5
    x = x_y_theta_t[-1][0] + delta_s * np.cos(x_y_theta_t[-1][2]*np.pi/180 + delta_theta * 0.5)
    y = x_y_theta_t[-1][1] + delta_s * np.sin(x_y_theta_t[-1][2]*np.pi/180 + delta_theta * 0.5)
    theta = x_y_theta_t[-1][2] + delta_theta*180/np.pi #FIX ME: theta needs to stay between -359 and +359 degrees
    
    x_y_theta_t.append([x, y, theta, t_ard])
```

The original equation is given in terms of distance, while our encoders gives a speed. We can however easily convert a speed to a distance by multiplying our speed with a Δt.  Let's look at the code in more detail:
```python
x_y_theta_t = [[0, 0, 0, 0]]
r_wheel = 0.025
wheel_sep = 0.210
```
The list x_y_theta_t contains our x-position, y-position, angle w.r.t. x-axis and the time. Since we assumed our robot starts at the origin of the odom frame the first element in this list is `[0, 0, 0, 0]`. We then define the wheel radius `r_wheel` in meter because we need to multiply our angular velocity with the wheel radius to get our linear velocity. We also define the distance between the two wheels `wheel_sep`because they're needed for the calculations.

```python
def calc_pos(msg):
    DIR_L = msg.data[0]
    wl = msg.data[1]
    DIR_R = msg.data[2]
    wr = msg.data[3]
    t_ard = msg.data[4]
```

Then we define our definition `calc_pos()` and assign the the contents of the topic we receive to their respective variables. 

***NOTE:** for the time being I don't do anything with the direction variables DIR_L and DIR_R because I don't know how I will indicate the direction. This is because I'm still having trouble getting the encoders to work.*

```python
    delta_theta = (-wl * r_wheel * (t_ard - x_y_theta_t[-1][3]) + wr * r_wheel * (t_ard - x_y_theta_t[-1][3])) / wheel_sep
    delta_s = (wl * r_wheel * (t_ard - x_y_theta_t[-1][3]) + wr * r_wheel * (t_ard - x_y_theta_t[-1][3])) * 0.5
```
Here we calculate Δθ and Δs. As stated before, the distance traveled is our linear velocity multiplied with a Δt. Δt is our current time minus the previous time. Our current time is in the message and our previous time is in the list x_y_theta_t.  We get our previous time in the list by writing `x_y_theta_t[-1][3]` 

```python
    x = x_y_theta_t[-1][0] + delta_s * np.cos(x_y_theta_t[-1][2]*np.pi/180 + delta_theta * 0.5)
    y = x_y_theta_t[-1][1] + delta_s * np.sin(x_y_theta_t[-1][2]*np.pi/180 + delta_theta * 0.5)
    theta = x_y_theta_t[-1][2] + delta_theta*180/np.pi #FIX ME: theta needs to stay between -359 and +359 degrees
```
Now that we have Δθ and Δs we can calculate , x, y and θ.
`np.cos()` and `np.sin()` need the angle in radians. The Δθ we calculated earlier is in radians, but for easy reading we will store our angle θ in degrees in our list x_y_theta_t.  This means we need to convert the θ from our list to radians. 
*NOTE: we know the Δθ we calculated earlier is in radians because when looking at the derivation of the equations you can see that they used s=rθ to get the distance traveled, which is only valid for a θ in radians.*

```python
    x_y_theta_t.append([x, y, theta, t_ard])
```

Finally we append the x-position, y-position, θ and the time to our list x_y_theta_t.

*At the time of writing this code I do not have the encoders working, so to test this code I simulated the encoder messages as a list. The python file I used for testing can be found at `proof of concept phase/testing-odometry-calculation.py`*

### Second iteration of python code
After looking at the previous code some more I realised that when our robot is fuly working, we wouldn't need to keep track of all the positions our robot has been. So I decided to rewrite the previous code a little bit. I also decided to use a numpy array instead of a list for reasons I do not know. For now I have only implented this is in the python file I used for testing. See `proof of concept phase/testing-odometry-calculation.py`

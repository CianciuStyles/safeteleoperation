## Overview ##

Safeteleoperation is a project developed by four students in [Master in Computer Engineering](http://cclii.dis.uniroma1.it/mce/) (Sapienza - Università di Roma) for the [Robot Programming Course](http://www.dis.uniroma1.it/~nardi/Didattica/CAI/robpro.html). The objective of the course is to introduce students to the usage of a robot programming framework, such as [Robot Operating System](http://www.ros.org/wiki/) and [OpenRDK](http://openrdk.sourceforge.net/), in order to develop a little project.

The main goals of this project are to provide:
  * safe teleoperation module:
    * based on distance map
    * based on gradient map
  * door trepassing module
  * Graphical User Interface, showing occupancy, distance, gradient and trajectory maps

<h2>Table of contents</h2>


## Dependencies ##
  * GNU/Linux system
  * [ROS](http://www.ros.org/wiki/)
  * [stage package](http://www.ros.org/wiki/stage)
  * [Qt and qmake tools](http://qt.nokia.com/products/)
  * [eros\_build](http://www.ros.org/wiki/eros_build)
  * [eros\_license](http://www.ros.org/wiki/eros_license)
  * joystick linux driver

## How to compile ##

To compile qtviewer node:
```
rosdep install qtviewer
roscd qtviewer
cmake -DQT_QMAKE_EXECUTABLE=/usr/share/qt4/bin/qmake .
rosmake qtviewer
```

Compile all the other nodes as usual in ROS.

## How to run ##

See the provided launchers:
  * [safe teleop + safe gate + DIS map](http://code.google.com/p/safeteleoperation/source/browse/trunk/dis-project-grad.launch)
  * [safe teleop dist + DIS map](http://code.google.com/p/safeteleoperation/source/browse/trunk/dis-project-dist.launch)

## ROS Packages ##
### Occupancy Map (occupancy\_map) ###
**Input:** [laser data](http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html) coming from the robot sensors and [odometry](http://www.ros.org/wiki/tf/Tutorials/Writing%20a%20tf%20listener%20(C%2B%2B))<br>
<b>Output:</b> occupancy map, represented as a vector of boolean values<br>
<p>Odometry is necessary in order to consistenly traslate the points of the occupancy map as the robot moves</p>

<h3>Distance Map (distance_map)</h3>
<b>Input:</b> occupancy map<br>
<b>Output:</b> distance map, represented as a vector of double values<br>
<p>The value contained in each cell of the vector represents the distance of that point from the closest obstacle.</p>

<h3>Gradient Map (gradient_map)</h3>
<b>Input:</b> distance map<br>
<b>Output:</b> gradient map, represented as two vectors of double values<br>
<p>For every cell, the values of the two vectors contain the intensity and orientation of the resulting force, considering the eight surrounding cells.</p>

<h3>Safe Teleoperation (safe_teleop)</h3>
This package provides three different nodes:<br>
<h4>Safe Gate (safe_gate)</h4>
<b>Input:</b> joystick commands, occupancy and distance maps<br>
<b>Output:</b> trajectory map and, when autopilot is enabled, self-created joystick commands to execute the computed path.<br>
<p>When the autopilot is enabled (through the press of one button of the joystick), the node disables the commands coming from the joystick, executes the A<code>*</code> algorithm to calculate a safe trajectory between the current position and the goal position (initially set to 5 meters ahead of the initial position) and triggers the commands necessary to follow the computed path.</p>
<h4>Safe Teleop (safe_teleop)</h4>
<b>Input:</b> joystick commands and gradient map<br>
<b>Output:</b> joystick commands modified according to a safe policy<br>
<br>
The node evaluates the force field around the robot and modifies the joystick commands according to it in order to avoid collisions. Furthermore, we used an <a href='http://sourceforge.net/projects/libjoyrumble/'>open-source library</a> in order to rumble the joystick when approaching an obstacle.<br>
<br>
<h4>Safe Teleop Distance (safe_teleop_distance)</h4>
<b>Input:</b> joystick commands and distance map<br>
<b>Output:</b> joystick commands modified according to a safe policy<br>
<br>
The node modifies the joystick commands according to the distance values of the cells around the robot in order to avoid collisions.<br>
<br>
<h3>Teleop Joy (teleop_joy)</h3>
See <a href='http://www.ros.org/wiki/joystick_drivers/Tutorials/TeleopNode'>ROS documentation</a>  about this package. The only modification we made is a repeat() function, in order to repeat inputs when a button remains pressed. This function is useful only when the device is not event-driven.<br>
<br>
<h3>Qt Viewer (qtviewer)</h3>

<b>Input:</b> occupancy, distance, gradient and trajectory maps<br>
<b>Output:</b> a graphical interface showing the maps
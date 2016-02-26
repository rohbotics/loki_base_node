# bus_server

[![Build Status](https://travis-ci.org/UbiquityRobotics/bus_server.svg?branch=master)](https://travis-ci.org/UbiquityRobotics/bus_server)

The bus_server module is one of many firmware side pieces of code used in the Ubiquity Robotics platforms that use a ROS Arduino Bridge subsystem such as Loki and Freya.

These commands are in general a subset of the official ROS Arduino Bridge commands with other commands added that use letters not defined by the official ROS Arduino Package as seen in ROS Hydra.

This readme is meant to document the commands that are received by bus_server and then processed here or in other modules.   The bus_server is the place where serial API is presented through the uart interface.


Firmware Commands For Ubiquity Robotics Implementation
-----------------
The single-letter commands over the serial port are used for polling sensors, controlling servos, driving the robot, and reading encoders.  These commands can be sent to the Arduino over any serial interface, including the Serial Monitor in the Arduino IDE.

**NOTE:** Before trying these commands, set the Serial Monitor baudrate to 115200 and the line terminator to "Carriage return" or "Both NL & CR" using the two pulldown menus on the lower right of the Serial Monitor window.



<pre>
Command Name   Cmd  Parms     Description
GET_BAUDRATE    b             Return baud rate (often used as a quick test or ping)
READ_ENCODERS   e             Return the current reading of the two wheel encoders
MOTOR_SPEEDS    m L R         Set motor speeds for left and right to be controlled by PID
PING            p S           Read one sonar sensor starting with sensor 1.  0 returns all.
RESET_ENCODERS  r             Reset encoder values both to 0
UPDATE_PID      u Kp Kd Ki Ko Update PID parameters with 
VERBOSE_DEBUG   v F           Set single bit flags to enable assorted debug modes 
MOTOR_DIRECT    z L R         Set motor speeds for left and right directly -126 to +126

Note: Commands of   a,c,d,s,t,w, and x are NOT supported in this firmware
</pre>


jenny
=====

An autonomous wheeled robot that solves mazes, avoids obstacles and counts objects on the way, and drops a package at a predetermined point.
The library functions we wrote while developing this robot are available here, open-source, for anyone to use in their 8051-based robotics projects. 

**apogee_main.c** :  Complete program for APOGEE - Track-o-mania; contains the latest working methods for sensor calibration, sensor reading, tag detection, wall avoidance, object counting, 
                 and package dropping, and PWM based speed control of DC motors.

**polulu_qtr8rc_read_calibrate.c** : contains functions for reading and calibrating Polulu's QTr-8RC reflectance sensor array with an 
                                 8051 microncontroller. Explanations of how to calibrate and integrate the methods with your program 
                                 are provided as comments in the source code.

**tryst_main.c** : Complete program for Tryst-Maze Buster; contains the methods for exploring the maze, calculating shortest path, and traversing
               maze along the shortest path; PWM based speed control of DC motors.


**Jenny is 8051 microcontroller based. Line sensing was done by a Polulu QTR-8C sensor, Object Sensing was done by IR transmitter-receiver pairs, movement by PWM controlled DC motors.**

Significant features developed are:
* Self written **library functions to calibrate and read** from the Polulu QTR-8RC sensor
* **Proportional control** of the robot's motion, PWM control of motors using 8051's inbuilt PWM timer
* **Maze solving** : Recording the path followed to reach the end of the maze and subsequently finding the shortest path.

Jenny was developed by Pranav N. Gour, Arun Subramaniyan and Sujay Narumanchi V. during the period Jan-March 2013. 

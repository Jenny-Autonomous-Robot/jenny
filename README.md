jenny
=====

An autonomous wheeled robot that solves mazes, avoids obstacles and counts objects on the way, and drops a package at a predetermined point

apogee_main.c :  Complete program for APOGEE - Track-o-mania; contains the latest working methods for sensor calibration, sensor reading, tag detection, wall avoidance, object counting, 
                 and package dropping, and PWM based speed control of DC motors.

polulu_qtr8rc_read_calibrate.c : contains functions for reading and calibrating Polulu's QTr-8RC reflectance sensor array with an 
                                 8051 microncontroller. Explanations of how to calibrate and integrate the methods with your program 
                                 are provided as comments in the source code.

tryst_main.c : Complete program for Tryst-Maze Buster; contains the methods for exploring the maze, calculating shortest path, and traversing
               maze along the shortest path; PWM based speed control of DC motors.

# Improvement of Potential Field Algorithm for Robot Path Planning
Author: Meshaal (Michel) Mouawad.

Email: mm4922@msstate.edu

Date: 09/13/2021

Mississippi State University, Electrical & Computer Engineering Department.

Initially proposed for real-time collision avoidance [Khatib 1986].  
Hundreds of papers published on APF
A potential field is a scalar function over the free space.
To navigate, the robot applies a force proportional to the 
negated gradient of the potential field.
A navigation function is an ideal potential field 
Southfield,Michigan
May 23, 2016

code was edited by Meshaal Mouawad 09/13/2021

# Objectives:
The objective of the project is to apply the Artificial Potential Field (APF) algorithm for robot path
planning learned during the class to improve this robot path planning algorithm and resolve some
issues such as local minima of an autonomous mobile robot.
Project Requirements

# Project Requirements
In the project, students are required to understand the Artificial Potential Field (APF) algorithm for
robot path planning and attempt to improve it

# Running the code

1. Navigate to the codes folder
2. Run the "PotentialFields.m" file for 1 obstacle environment and "PotentialFields_Q2.m" for 2 obstacle environment.

Note: other files are helper files for the above main scripts

File #1 ECE8743_PotentialFields_Obstacle_1.m
This file for the simulation of one obatacles

File ECE8743_PotentialFields_Obstacles_2.m
This file for the simulation of 2 obatacles.

file: section_5.m
this is the code for the 8 obstacles.

file: section_6.m
this file has the code for the local minima.


in section 7 folder there is the simualtion of the improved APF
Folder: section_7 > file: GoalDelta.m 
This is the code for the improved APF for finsing the delta of the goal.

Folder: section_7 > file: ObsDelta.m
This is the code for the improved APF for finsing the delta of the obstacle

# References


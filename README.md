# Lego EV3 Obstacle Avoidance Rover | AER627 – Space Robotics
This project involved the design, programming, and demonstration of a LEGO EV3 two-wheel differential rover for autonomous obstacle avoidance. The rover navigates toward a goal while avoiding obstacles using a hybrid of Potential Field and A* path planning, with a Proportional-Derivative (PD) controller executing the motion along a sparsified path.

## Project Overview
- Autonomous two-wheel LEGO EV3 rover
- Obstacle avoidance using Potential Field and A* planners
- AprilTag-based scene recognition and mapping
- PD heading controller with MATLAB integration
- Real-world testing with calibration and friction challenges

## Contents
- [Project Report (PDF)](./627_project4_report.pdf) – Full write-up of design, theory, testing, tuning, and results
- [Demo Video (MP4)](./project4_demo.mp4) – Real-world demonstration of rover path execution and avoidance
- [MATLAB Code](./code_v3.m) – All source files including gradient planning, controller logic, and robot execution

## Rover

A simple design for the rover was used, featuring 2 wheels both connected to their own motors, along with a stabilizing
steel ball at the back providing structure and smoothing movement. An image of the design can be seen below.

![Rover](Images_and_Plots/rover.png)  
*Rover Design*

## Apriltags and Scene Generation

Four Apriltags (barcode-like images) were used to generate a scene. A photo is taken of this scene, rendered and processed into the MATLAB program, and
a 2 rover paths are produced. One path using the generation of a potential field, the other using A* path planning. The purpose for each barcode is described below:  

apriltag 1: set the origin of the scene  
apriltag 2: set the location of the rover  
apriltag 3: obstacle 1  
apriltag 4: obstacle 2  

It is worth noting the start position is the initial position of the rover, while the end position (or end goal defining the end of the planned paths) is hard coded
once the scene has been rendered into the program. The following figures show the scene and its rendering after having been processed in MATLAB.  

![scene](Images_and_Plots/scene.png)  
*Scene Processed in MATLAB - photo taken using Iphone camera*

![Rendered Scene](Images_and_Plots/rendered_scene.png)  
*Rendered Scene by MATLAB*

## Authors
- Elliott Arpino  
- Octavio Guerra  

## Course Info
- AER627 – Introduction to Space Robotics, Winter 2025  
- Instructor: Dr. Anton de Ruiter  
- Toronto Metropolitan University

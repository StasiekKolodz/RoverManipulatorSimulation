# Rover Manipulator Simulation

This software allows you to simulate the rover's manipulator and control it with the hardware operator panel via an Ethernet connection.
This code is part of the operator panel project for the Mars Rover developed by the Students Robotics Association KNR.

Received data from the panel:\
-Joystick position \
-Buttons \
-Potentiometer 

# ROS2 packages

### JoystickBridge
This package handles communication with the operator panel via ethernet. Data from the panel is received in the KNR rover custom message (refer to KNR rover docs).

### ReverseKinematics
This package handles reverse kinematics problems to visualize the manipulator position later (manip is controlled via gripper xyz position, not in joint space).

### dobot_viz
Visualization of the manipulator in Rviz(based on dobot physical parameters)

## Run Simulation
````bash
ros2 launch rover_sim dobot_viz.launch.py
````

# Prerequisities

## ROS2 - Humble
ROS2 Humble requires Ubuntu 22.04. Make sure the correct version is installed according to [official tutorial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

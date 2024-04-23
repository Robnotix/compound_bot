# Compound Bot

## [Robot Control](robot_control/README.md)
Drivers for controlling the MyAGV and the MyCobot robots from Elephant Robotics

## [Sensing](sensing/README.md)
Drivers for getting data from the onboard sensors

## Teleoperation

### [API (In Progress)](teleoperation/api/README.md)
Web server with REST and Websocket endpoints for getting sensor data and moving the robot

### Client (TODO)
Client library that interfaces with the API

### [GUI (TODO)](teleoperation/gui/README.md)
Human interface for controlling the robot remotely

### ROS Workspace (TODO):
So far this is a pure python implementation. Eventually it would be great to port the functionalities into ROS2 nodes.
This would allow us to use the following ROS tools:
- [Cartographer](https://google-cartographer-ros.readthedocs.io/en/latest/) for building maps for AGV navigation
- [Nav2](https://navigation.ros.org/) for autonomous AGV nagivation
- [Moveit](https://moveit.ros.org/) for robot arm path planning
- [Gazebo](https://gazebosim.org/home) for physics based simulation and empowering machine learning in simulation
- [Foxglove]() and Mcap for logging of structured data and easy visualization

# BOM:
- 1X - [MyCobot280](https://shop.elephantrobotics.com/collections/mycobot-280/products/mycobot-worlds-smallest-and-lightest-six-axis-collaborative-robot)
- 1X - [MyAgv](https://shop.elephantrobotiTelecs.com/collections/myagv-smart-navigation-robot/products/myagv-2023-pi)
- 1X - [OAKD-Lite depth Camera](https://shop.luxonis.com/products/oak-d-lite-1)
- 1X - [3D Printed Parts Set](https://foxglove.dev/) (TODO: add STL files to repo)

# Setup:

TODO: Add setup docs

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
- Cartographer for building maps for AGV navigation
- Nav2 for autonomous AGV nagivation
- Moveit for robot arm path planning
- Gazebo for physics based simulation and empowering machine learning in simulation
- Foxglove and Mcap for logging of structured data and easy visualization

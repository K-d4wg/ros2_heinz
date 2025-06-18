
# Unitree H1 Simulation and Control

A collection of ROS2 Packages to handle the simulation of the Unitree H1 Quadruped robot using ROS2 Jazzy and Gazebo Sim Harmonic. This repository includes everthing necessary (IMU-/ Contact-Sensors, ROS2 Interface, ...) to simulate and control the robot using joint space commands (position or torque). High level control (like locomotion) is currently under development.
## Installation


Important Packages:
- numpy, scipy, matplotlib
- a recent version of Pygame (for Kinematics Demo GUI / Locomotion Controller, but "/pose" and "/cmdvel" topics can also be published via commandline or RQT!)
- ROS2 Jazzy
- Gazebo Sim Harmonic (older distributions of Gazebo Sim do not fully support e.g. the Feet Contact Sensors)

Installation:
- in /ros2_ws/src
- git clone this repository
- build the packages using colcon 
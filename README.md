
# Unitree H1 Simulation and Control

A collection of ROS2 Packages to handle the simulation of the Unitree H1 robot using ROS2 Jazzy and Gazebo Sim Harmonic. This repository includes everything necessary (Sensors, ROS2 Interface, ...) to simulate and control the robot using joint space commands (position or torque).

## Usage

### Position Control
To launch the H1 Simulation in Position Control Mode use:
```bash
ros2 launch ros_gz_h1_bringup h1_gazebo_sim.launch.py
```
The robot will be standing upright and after unpausing the simulation, you can send joint commands using the ROS2 topics:

/h1/<joint_name>/cmd_pos

### Torque Control
To launch the H1 Simulation in Torque Control Mode use:
```bash
ros2 launch ros_gz_h1_bringup h1_gazebo_sim_torque_ctrl.launch.py
```

The robot will collapse in on itself upon start since no torques are applied to keep it upright. To control the robot, use the ROS2 topics:

/<joint_name>_cmd

## Installation
Important Packages:
- ROS2 Jazzy
- Gazebo Sim Harmonic (older distributions of Gazebo Sim do not fully support e.g. the Feet Contact Sensors)
- ros-jazzy-joint-state-publisher-gui package (you can install it with apt)

Installation:
- in /ros2_ws/src
- git clone this repository
- build the packages using colcon 

![Alt Text](https://media3.giphy.com/media/v1.Y2lkPTc5MGI3NjExeHJteHFyc2NuOTcxY3N4amk2aWh6dnRucHo0emFlMGM0M2trOWc4NiZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/N2HGf1zrWOn8Hs787f/giphy.gif)

## Why Heinz?
"1" is "Eins" in German. So H1 becomes Heins ~ Heinz (a common German first name)

## Credits
- Project Template: [ros_gz_project_template](https://github.com/gazebosim/ros_gz_project_template)
- Originally part of course-work for the lecture 
"Humanoid Robots - Locomotion and Whole-Body Control" at KIT, held by Prof. Dr. Katja Mombaur and Dr. Jonas Große Sundrup
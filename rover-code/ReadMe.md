# Rover Code

## Overview

This repository contains code for the rover project, divided into two parts: ROS integration and embedded systems. The ROS code runs on a Jetson Nano for high-level control and sensor processing, while the embedded code runs on Teensy 4.1 microcontrollers to handle low-level hardware operations.

## Folder Structure

- `ros/`: Contains the ROS Melodic architecture, designed for Jetson Nano, responsible for sensor fusion, navigation, and communication.
- `embedded/`: Contains C++ code for the Teensy 4.1 microcontrollers, which manage motor control and other hardware components.

## Getting Started

### Prerequisites

- ROS Melodic installed on the Jetson Nano
- PlatformIO or Arduino IDE for programming the Teensy 4.1 microcontrollers

### Setup Instructions

1. **Jetson Nano (ROS setup)**:
   - Navigate to the `ros/` folder and build the ROS workspace:
     ```bash
     cd ros
     catkin_make
     source devel/setup.bash
     ```
   - Launch the necessary ROS nodes to manage sensor integration and control.

2. **Teensy 4.1 (Embedded Setup)**:
   - Open the `embedded/` folder in PlatformIO or Arduino IDE.
   - Ensure the Teensy 4.1 microcontrollers are connected and upload the respective C++ code.

## Usage

- **Jetson Nano (ROS)**: Handles navigation, path planning, and processing sensor data.
- **Teensy Microcontrollers**: Controls real-time motor operations and collects data from sensors.

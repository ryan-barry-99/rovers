# ROS Code for Rover

## Overview

This folder contains the ROS Melodic architecture for the rover, designed to run on the Jetson Nano. It manages high-level communication between the rover's subsystems via ROS topics, services, and actions. It interfaces with the Teensy microcontrollers through UART to control various hardware components on the rover.

## Folder Structure

- `communications/`: Handles the UART interface to the CAN network. This includes communication between the Jetson Nano (ROS) and the embedded Teensy controlling the rover subsystems.
  
- `constants/`: Defines rover-specific and CAN-related constants for system-wide use.

- `hardware/`:
  - `sensors/`: Contains drivers for various sensors, including cameras and IMU.
  - `subsystems/`: Contains the kinematics for the arm and the drive base.

## Subsystems Overview

- **Communications (UART to CAN)**: Facilitates message handling between the CAN network and the ROS network.
- **Kinematics**: Manages calculations for the rover's arm and drive base, ensuring accurate movement and control.

## Getting Started

1. Build the ROS workspace:
   ```bash
   cd ros
   catkin_make
   source devel/setup.bash
# Rovers

This repository contains the source code for RIT's University Rover Challenge rover.

The software and firmware architecture is broken down into the following components:

- Python high level ROS application
    Responsible For:
    - Arm kinematics
    - Base station communication
    - Camera control
    - Drive base kinematics
    - Perception

- Embedded Firmware
    - Main Body Board
        Responsible For:
        - Drive base control
        - Temperature control

    
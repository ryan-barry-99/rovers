#!/usr/bin/env python3
"""
File: rover_main.py

Description: This script serves as the main control module for a rover system. 
It utilizes the Robot Operating System (ROS) client library (rclpy) to interface 
with the rover's hardware components. The program defines a MissionControl class 
responsible for managing various missions that the rover can execute. Each mission 
is represented by a separate class: AutonomousNavigationMission, 
EquipmentServicingMission, ExtremeRetrievalDeliveryMission, and ScienceMission.

Author: Ryan Barry
Date Created: July 16, 2023
"""

import rclpy
from communications.XBox_Controller import XboxController
from hardware.Rover import Rover
from hardware.RoverConstants import *
from missions.AutonomousNavigation import AutonomousNavigationMission
from missions.EquipmentServicing import EquipmentServicingMission
from missions.ExtremeRetrievalDelivery import ExtremeRetrievalDeliveryMission
from missions.ScienceMission import ScienceMission


class MissionControl:
    def __init__(self):
        self.rover = Rover()
        self.autonomous_navigation = AutonomousNavigationMission(self.rover)
        self.equipment_servicing = EquipmentServicingMission(self.rover)
        self.extreme_retrieval_delivery = ExtremeRetrievalDeliveryMission(self.rover)
        self.science_mission = ScienceMission(self.rover)
        self.controller = XboxController()

    def exec(self):
        self.controller.publish_controller_state()

        mission = self.rover.get_mission()

        if mission == AUTONOMOUS:
            self.autonomous_navigation.run()
        if mission == EQUIPMENT_SERVICING:
            self.equipment_servicing.run()
        if mission == EXTREME_RETRIEVAL_DELIVERY:
            self.extreme_retrieval_delivery.run()
        if mission == SCIENCE:
            self.science_mission.run()


def main(args=None):
    rclpy.init(args=args)
    mission_control = MissionControl()

    while rclpy.ok():
        mission_control.exec()
        rclpy.spin_once(mission_control.controller)

    mission_control.controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

import rclpy
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

    def exec(self):
        mission = self.rover.active_mission

        if mission == AUTONOMOUS:
            self.autonomous_navigation.run()
        if mission == EQUIPMENT_SERVICING:
            self.equipment_servicing.run()
        if mission == EXTREME_RETRIEVAL_DELIVERY:
            self.extreme_retrieval_delivery.run()
        if mission == SCIENCE:
            self.science_mission.run()


if __name__ == '__main__':
    mission_control = MissionControl()
    mission_control.exec()
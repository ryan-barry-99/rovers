#!/usr/bin/env python3
"""
File: SciencePlate.py

Description:

Author: Ryan Barry
Date Created:
"""
import sys
sys.path.append("../..")

from subsystems.cameras.ScienceCamera import ScienceCamera
from sensors.Fluorometer import Fluorometer
from sensors.Humidity import Humidity
from HeatBlock import HeatBlock
from Backpack import Backpack
from LipidDetector import LipidDetector
from DNADetector import DNADetector

class SciencePlate:
    def __init__(self):
        self.camera = ScienceCamera()
        self.fluorometer = Fluorometer()
        self.humidity_sensor = Humidity()
        self.heat_block = HeatBlock()
        self.backpack = Backpack()
        self.lipid_detector = LipidDetector()
        self.dna_detector = DNADetector()
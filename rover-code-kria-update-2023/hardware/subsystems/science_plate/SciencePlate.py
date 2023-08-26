"""
File: SciencePlate.py

Description:

Author: Ryan Barry
Date Created:
"""
import sys
sys.path.append("..")
from cameras.ScienceCamera import ScienceCamera

class SciencePlate:
    def __init__(self):
        self.camera = ScienceCamera()

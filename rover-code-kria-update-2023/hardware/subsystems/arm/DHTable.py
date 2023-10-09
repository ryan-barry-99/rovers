"""
File: DHTable.py

Description: This module defines the DH table class used to compute
forward kinematics for an arm robot.

Author: Ryan Barry
Date Created: October 9, 2023
"""
import numpy as np

class DHTable:
    def __init__(self, links):
        self.setLinks(links)
        
    def setLinks(self, links):
        self.links = links
        self.__theta = np.zeros(1, len(self.links))  # Initialize theta as a np array of zeros
        self.__d = np.zeros(1, len(self.links))  # Initialize d as a np array of zeros
        self.__a = np.zeros(1, len(self.links))  # Initialize a as a np array of zeros
        self.__alpha = np.zeros(1, len(self.links))  # Initialize alpha as a np array of zeros
        self.update()
        
    def update(self):
        for i, link in enumerate(self.links):
            self.__theta[i] = link.theta
            self.__a[i] = link.a
            self.__d[i] = link.d
            self.__alpha[i] = link.alpha
        
    def theta(self, index):
        return self.__theta(index)
    
    def d(self, index):
        return self.__d(index)
    
    def a(self, index):
        return self.__a(index)
    
    def alpha(self, index):
        return self.__alpha(index)
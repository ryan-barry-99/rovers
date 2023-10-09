"""
File: ArmRobotKinematics.py

This module defines the ArmRobot class, which represents an arm robot with multiple joints.
It provides functionality to add links to the arm, move individual joints, update the
Denavit-Hartenberg (DH) table, and perform forward kinematics to compute the end-effector
position and orientation.

Author: Ryan Barry
Date created: June, 26 2023
"""

import numpy as np  # Importing the NumPy library for mathematical operations
from Link import PRISMATIC, REVOLUTE, Link
from DHTable import DHTable


class ArmRobotKinematics:
    def __init__(self):
        # self.num_joints = 0  # Initialize the number of joints to 0
        self.links = []
        self.dhTable = DHTable(self.links)  # Initialize DH table

    # Add a link to the arm
    # Parameters:
    #   joint_type:
    #       - REVOLUTE
    #       - PRISMATIC
    #   length (meters)
    #   alpha (radians)
    #       - rotate around the x1 axis an angle alpha to make z1 parallel to z2
    def addLink(self, joint_type, length, alpha):
        new_link = Link(joint_type=joint_type, length=length, alpha=alpha)
        self.links.append(new_link)
        self.dhTable.setLinks(self.links)
        return new_link

    def updateDHTable(self):
        self.dhTable.update()
        # Compute the forward kinematics based on the updated DH Table and return the transformation matrix
        return self.forward_kinematics()

    def forward_kinematics(self):
        self.__A = []  # Initialize A matrices as empty list
        self.__T = np.identity(4)  # Initialize the transformation matrix as an identity matrix

        for i, link in enumerate(self.links):
            theta = self.dhTable.theta(i)  # Get the joint angle
            d = self.dhTable.d(i)  # Get the d parameter
            a = self.dhTable.a(i)  # Get the a parameter
            alpha = self.dhTable.alpha(i)  # Get the alpha parameter
            
            ct = np.cos(theta)  # Compute the cosine of theta
            st = np.sin(theta)  # Compute the sine of theta
            ca = np.cos(alpha)  # Compute the cosine of alpha
            sa = np.sin(alpha)  # Compute the sine of alpha

            if link.joint_type == REVOLUTE:  # Check if the joint type is revolute
                self.__A.append(
                    np.array(
                        [
                            [ct, -st * ca, st * sa, a * ct],  # Create the transformation matrix A
                            [st, ct * ca, -ct * sa, a * st],
                            [0, sa, ca, d],
                            [0, 0, 0, 1]
                        ]
                    )
                )

            elif link.joint_type == PRISMATIC:  # Check if the joint type is prismatic
                self.__A.append(
                    np.array(
                        [
                            [ct, -st * ca, st * sa, ct * d],  # Create the transformation matrix A
                            [st, ct * ca, -ct * sa, st * d],
                            [0, sa, ca, a],
                            [0, 0, 0, 1]
                        ]
                    )
                )
            else:
                print(f"Invalid joint type at joint {i}")
                return None

            self.__T = np.dot(self.__T, self.__A[i])  # Multiply the transformation matrix T by A

        return self.__T  # Return the final transformation matrix

    def inverse_kinematics(self, target_position, target_orientation):
        pass

#!/usr/bin/env python

"""
    Navigation.
"""

# ROS and OpenCV packages
from __future__ import division
import rospy
import time
import math

# Mobile base velocity message type
from geometry_msgs.msg import Twist, Vector3

class Navigation:

    def __init__(self):
        """ Class constructor """
        # TODO: Initialise class

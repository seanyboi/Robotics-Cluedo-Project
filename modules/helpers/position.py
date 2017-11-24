#!/usr/bin/env python

"""
    Script to position robot
    in front of the image.
"""

# ROS and OpenCV packages
from __future__ import division
import numpy as np
import rospy
import math
import time
import tf

# Topics messages types
from geometry_msgs.msg import Twist, Vector3
from ar_track_alvar_msgs.msg import AlvarMarkers

# Find desired point
def quaternion_to_euler():

    # tf listener
    listener = tf.TransformListener()
    time.sleep(3)

    # Get ar marker tranformation matrix (respect to the map)
    (trans, rotation) = listener.lookupTransform('/map', '/ar_marker_0', rospy.Time(0))

    # Build translation rotation matrix
    matrix = listener.fromTranslationRotation(trans, rotation)

    # Get z column in the matrix
    direction = matrix[:2 , 2]

    # Compute desired point (in front of ar_marker)
    pose = trans[:2] + direction * 0.6

    # Get desired robot rotation
    theta = math.atan2(pose[1], pose[0])

    return pose, theta

#!/usr/bin/env python

'''
    Object recognition.

    The class makes use of the PlaneTracker
    to find features points (salient points)
    and carry on the recognition of the image
    given.
'''

# Python 2/3 compatibility
from __future__ import print_function

import numpy as np
import rospy
import cv2
import os

# modules
from geometry_msgs.msg import Twist, Vector3
from helpers import PlaneTracker, toMAT, temp_matching

class Recognition:

    def __init__(self):
        """ Class constructor """

        # Recognition counter
        self.counter = 0

        # Velocity object
        self.velocity = Twist()

        # Flag
        self.recognised = False

        # PlaneTracker object
        self.tracker = PlaneTracker()

        # Load characters
        plum    = cv2.imread(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'data/images/plum.png')))
        mustard = cv2.imread(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'data/images/mustard.png')))
        peacock = cv2.imread(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'data/images/peacock.png')))
        scarlet = cv2.imread(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'data/images/scarlet.png')))

        # Load weapons
        rope     = cv2.imread(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'data/images/rope.png')))
        wrench   = cv2.imread(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'data/images/wrench.png')))
        revolver = cv2.imread(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'data/images/revolver.png')))

        # Characters rect
        plum_rect    = (0, 0, plum.shape[1], plum.shape[0])
        mustard_rect = (0, 0, mustard.shape[1], mustard.shape[0])
        peacock_rect = (0, 0, peacock.shape[1], peacock.shape[0])
        scarlet_rect = (0, 0, scarlet.shape[1], scarlet.shape[0])

        # Weapons rect
        rope_rect     = (0, 0, rope.shape[1], rope.shape[0])
        wrench_rect   = (0, 0, wrench.shape[1], wrench.shape[0])
        revolver_rect = (0, 0, revolver.shape[1], revolver.shape[0])

        # Add characters to plan tracker
        self.tracker.add_target(plum, plum_rect, "plum")
        self.tracker.add_target(mustard, mustard_rect, "mustard")
        self.tracker.add_target(peacock, peacock_rect, "peacock")
        self.tracker.add_target(scarlet, scarlet_rect, "scarlet")

        # Add weapons to plan tracker
        self.tracker.add_target(rope, rope_rect, "rope")
        self.tracker.add_target(wrench, wrench_rect, "wrench")
        self.tracker.add_target(revolver, revolver_rect, "revolver")

        # Mobile base velocity publisher
        self.velocity_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size = 10)

    def recognise(self, raw_image):
        """
            Runs the PlaneTracker track
            method to find the best match.

            Arguments:
                param1: RBG raw image

            Returns:
                string: The name of matched object
        """
        print("Running recognition...")

        # RGB raw image to OpenCV bgr MAT format
        img = toMAT(raw_image)

        # Convert to grayscale
        img_greyscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Correct image exposure
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        cl1 = clahe.apply(img_greyscale)

        tracked = self.tracker.track(img_greyscale)

        if len(tracked) > 0:

            print("Image found and saved")

            for tracked_ob in tracked:

                res = tracked_ob.target.data
                print("Found: ", res)
                self.recognised = True
                tmpmtch_save = temp_matching(res, toMAT(raw_image))
                print(res, tmpmtch_save.shape)
                return True, res, tmpmtch_save

        else:
            print("Image not detected, getting closer...")

            if self.counter < 3:

                # Get robot closer to image
                self.velocity.angular.z = 0
                self.velocity.linear.x = 0.08
                self.velocity_pub.publish(self.velocity)

                # Sleep
                rospy.sleep(2)

                # Try recognition again
                self.recognise(raw_image)

                # Increase counter
                self.counter += 1
                print("Recognition counter: ", self.counter)

            else:
                self.counter = 0
                return False, False, False

    def is_recognised(self):
        """
            Getter method used by the main
            to check whether the image has
            been recognised or not, to move
            on to the next image.

            Returns:
                bool: True or False (recognised or not)
        """
        return self.recognised

    def reset_rec_flag(self):
        """
            Resets recognies flag
            for new search.
        """
        self.recognised = False

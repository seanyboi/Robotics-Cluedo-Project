#!/usr/bin/env python

'''
Feature homography
==================

Example of using features2d framework for video homography matching.
ORB features and FLANN matcher are used. The actual tracking is implemented by
PlaneTracker class in plane_tracker.py

Inspired by http://www.youtube.com/watch?v=-ZNYoL8rzPY

video: http://www.youtube.com/watch?v=FirtmYcC0Vc

Usage
-----
feature_homography_from_file1.py [<video source>]

'''

# Python 2/3 compatibility
from __future__ import print_function

import numpy as np
import cv2
import os

# modules
from plane_tracker import PlaneTracker

class Recognition:

    # Constructor
    def __init__(self):

        # Descriptor tracker
        self.tracker = PlaneTracker()

        # Set tracker
        self.setTracker()

    # Set plane tracker
    def setTracker(self):

        # Load characters
        plum    = cv2.imread(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', '..', 'data/images/plum.png')))
        mustard = cv2.imread(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', '..', 'data/images/mustard.png')))
        peacock = cv2.imread(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', '..', 'data/images/peacock.png')))
        scarlet = cv2.imread(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', '..', 'data/images/scarlet.png')))

        # Load weapons
        rope     = cv2.imread(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', '..', 'data/images/rope.png')))
        wrench   = cv2.imread(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', '..', 'data/images/wrench.png')))
        revolver = cv2.imread(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', '..', 'data/images/revolver.png')))

        # Characters rect
        plum_rect    = (0, 0, plum.shape[1], plum.shape[0])
        mustard_rect = (0, 0, mustard.shape[1], mustard.shape[0])
        peacock_rect = (0, 0, peacock.shape[1], peacock.shape[0])
        scarlet_rect = (0, 0, scarlet.shape[1], scarlet.shape[0])

        # Weapons rect
        rope_rect     = (0, 0, rope.shape[1], rope.shape[0])
        wrench_rect   = (0, 0, wrench.shape[1], wrench.shape[0])
        revolver_rect = (0, 0, revolver.shape[1], revolver.shape[0])

        # Add characters to plan
        self.tracker.add_target(plum, plum_rect, "plum")
        self.tracker.add_target(mustard, mustard_rect, "mustard")
        self.tracker.add_target(peacock, peacock_rect, "peacock")
        self.tracker.add_target(scarlet, scarlet_rect, "scarlet")

        # Add weapons to plan
        self.tracker.add_target(rope, rope_rect, "rope")
        self.tracker.add_target(wrench, wrench_rect, "rrench")
        self.tracker.add_target(revolver, revolver_rect, "revolver")

    # Run recognition
    def track(self, img):

        tracked = self.tracker.track(img)

        if len(tracked) > 0:
            for tracked_ob in tracked:
                return tracked_ob.target.data

                # Homography info
                # h, status = cv2.findHomography(tracked_ob.p0, tracked_ob.p1)
        else:
            return None

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
        self.tracker.add_target(wrench, wrench_rect, "rrench")
        self.tracker.add_target(revolver, revolver_rect, "revolver")

    # Run recognition
    def run(self, img):

        # Find recognised object
        tracked = self.tracker.track(img)

        # Send recognition result
        if len(tracked) > 0:
            for tracked_ob in tracked:
                return tracked_ob.target.data

                # Homography info
                # h, status = cv2.findHomography(tracked_ob.p0, tracked_ob.p1)
        else:
            return None

    # Converts image into MAT format
    def recognise(self, data):

        # Run image recognition
        res = self.recognition.track(image)

        # Check recognition result
        # if negative adjust robot
        # position and try again
        if res is not None:

            print("Image found and saved")

            # Save image under detections
            cv2.imwrite(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'data/detections/%(res)s.png' % locals())), image)

            try:
                # Collect tf data
                time.sleep(3)

                # Write to file (position of the image)
                file = open(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'data/poses.txt')), "w")
                file.write('%(res)s: ' % locals() + str(self.tf_listener.lookupTransform('/map', '/ar_marker_0', rospy.Time(0))[0]))
                file.close()

            except Exception as e:
                print("Error while writing image pose: ", e)

    def is_recognised(self):
        return self.recognised

#!/usr/bin/env python

'''
Feature homography
==================

Example of using features2d framework for video homography matching.
ORB features and FLANN matcher are used. The actual tracking is implemented by
PlaneTracker class in plane_tracker.py

'''

# Python 2/3 compatibility
from __future__ import print_function

import numpy as np
import cv2
import os
import tf
import rospy

# modules
from helpers import PlaneTracker, toMAT

class Recognition:

    # Constructor
    def __init__(self):

        # Flag
        self.recognised = False

        # Tf object
        self.tf_listener = tf.TransformListener()

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

    # Converts image into MAT format
    def recognise(self, raw_image):

        # RGB raw image to OpenCV bgr MAT format
        img = toMAT(raw_image)

        tracked = self.tracker.track(img)

        if len(tracked) > 0:

            print("Image found and saved")

            for tracked_ob in tracked:

                res = tracked_ob.target.data
                print("Found: ", res)

                try:

                    # Save image under detections
                    cv2.imwrite(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'data/detections/%(res)s.png' % locals())), img)

                    # Collect tf data
                    rospy.sleep(3)

                    # Write to file (position of the image)
                    file = open(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'data/poses.txt')), "w")
                    file.write('%(res)s: ' % locals() + str(self.tf_listener.lookupTransform('/map', '/ar_marker_0', rospy.Time(0))[0]))
                    file.close()

                    self.recognised = True

                    return res

                except Exception as e:
                    print("Error while writing image pose: ", e)

        else:
            print("Image not found")

    def is_recognised(self):
        return self.recognised

    def reset_rec_flag(self):
        self.recognised = False

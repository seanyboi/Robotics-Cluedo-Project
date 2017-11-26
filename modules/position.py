#!/usr/bin/env python

"""
    Position.
"""

# ROS and OpenCV packages
from __future__ import division
import cv2
import rospy
import time
import math
import tf
import os
import numpy as np

# Messages
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError

# Routines
from gotopose import GoToPose
from helpers import blur, greyscale, threshold, morph, canny

class Position:

    def __init__(self):
        """ Class constructor """

        # Camera size
        self.x = 640
        self.y = 480

        # Object instances
        self.velocity = Twist()
        self.bridge = CvBridge()
        self.gotopose = GoToPose()
        self.tf_listener = tf.TransformListener()

        # Flags
        self.img_centered = False
        self.ar_positioned = False

        # Rate
        self.rate = rospy.Rate(10)

        # Mobile base velocity publisher
        self.velocity_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size = 10)

    # Get in front of the AR
    def toAR(self, raw_image):

        if not self.ar_positioned:

            # Get ar marker tranformation matrix (respect to the map)
            rospy.sleep(3)
            (trans, rotation) = get_ar_transform()

            # Get rotation matrix
            matrix = self.tf_listener.fromTranslationRotation(trans, rotation)

            # Get z column in the matrix
            direction = matrix[:3 , 2]

            # Compute desired point (in front of ar_marker)
            pose = trans + direction * 0.4

            # Get desired robot rotation
            theta = math.atan2(pose[1], pose[0])

            # Build desired pose
            x, y = pose[0], pose[1]
            position = {'x': x, 'y' : y}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

            # Send robot to pose
            rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
            success = self.gotopose.goto(position, quaternion)

            if success and get_ar_transform()[0]:

                rospy.loginfo("Given map position reached")

                # Start image centering
                self.ar_positioned = True
                center_image(img_data)

            elif success and not get_ar_transform()[0]:

                rospy.loginfo("Given map position reached but AR marker offset... Starting recover procedure")

            else:
                rospy.loginfo("Failure in reaching given position")

                # Try new positioning
                self.ar_positioning = False

            # Sleep to give the last log messages time to be sent
            rospy.sleep(1)

    def center_image(self, raw_image):

        try:
            # RGB raw image to OpenCV bgr MAT format
            image = self.bridge.imgmsg_to_cv2(raw_image, 'bgr8')

            # Apply convolutional kernel to smooth image
            blurred = blur(image)

            # Convert image to greyscale
            grey = greyscale(blurred)

            # Apply adaptive thresholding
            thresh = threshold(grey)

            # # Show image
            # cv2.namedWindow('Thresholded Image')
            # cv2.imshow('Thresholded Image', thresh)
            # cv2.waitKey(5)

            # Canny edge detector
            edges = canny(thresh)

            # # Show image
            # cv2.namedWindow('Canny')
            # cv2.imshow('Canny', edges)
            # cv2.waitKey(5)

            # Find the contours
            (contours, _) = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Compute areas for contours found
            cnt_areas = [cv2.contourArea(contours[n]) for n in range(len(contours))]

            # Get biggest contour index
            maxCnt_index = cnt_areas.index(max(cnt_areas))

            # Draw contours
            cv2.drawContours(image, contours, maxCnt_index, (0,255,0), 3)

            # Show image
            cv2.namedWindow('Contours')
            cv2.imshow('Contours', image)
            cv2.waitKey(5)

            # Contour pose (for centering purposes)
            M = cv2.moments(contours[maxCnt_index])
            max_cnt_x = int(M['m10']/M['m00'])
            max_cnt_y = int(M['m01']/M['m00'])

            # Centre image within the frame
            if self.x - max_cnt_x > 330:
                self.velocity.angular.z = 0.2

            elif self.x - max_cnt_x < 300:
                self.velocity.angular.z = -0.2

            else:

                # Stop rotation (centering completed)
                self.velocity.angular.z = 0
                self.img_centered = True

            # Publish velocity
            self.velocity_pub.publish(self.velocity)

        except Exception as CvBridgeError:
            print('Error during image conversion: ', CvBridgeError)

    def get_ar_transform(self):
        return self.tf_listener.lookupTransform('/map', '/ar_marker_0', rospy.Time(0))

    def is_positioned_completed(self):
        return self.ar_positioned and self.img_centered

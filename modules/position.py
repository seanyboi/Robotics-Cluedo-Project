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
        self.gtp = GoToPose()
        self.tf_listener = tf.TransformListener()

        # Flags
        self.img_centered = False
        self.ar_positioned = False

        # Rate
        self.rate = rospy.Rate(10)

        # Mobile base velocity publisher
        self.velocity_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size = 10)

    # Get in front of the AR
    def toAR(self):

        # Get ar marker tranformation matrix (respect to the map)
        (trans, rotation) = get_ar_transform()

        # Get rotation matrix
        matrix = self.tf_listener.fromTranslationRotation(trans, rotation)

        # Get z column in the matrix
        direction = matrix[:3 , 2]

        # Compute desired point (in front of ar_marker)
        pose = trans + direction * 0.4

        # Get desired robot rotation
        theta = math.atan2(pose[1], pose[0])

        # Send robot to pose
        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = self.gtp.goto(pose[0], pose[1], theta)

        if success and get_ar_transform()[0]:

            rospy.loginfo("Given map position reached")

            # Start image centering
            self.ar_positioned = True

        elif success and not get_ar_transform()[0]:

            rospy.loginfo("Given map position reached but AR marker offset... Starting recover procedure")

            # Rotate robot and check for AR marker
            while not rospy.is_shutdown() and not get_ar_transform()[0]:

                # Rotation
                self.velocity.linear.x = 0
                self.velocity.angular.z = getRadians(15)

                # Rotate
                for x in range(30):
                    pub.publish(desired_velocity)
                    self.rate.sleep()

            rospy.loginfo("Successful recovering !")

            # Start image centering
            self.ar_positioned = True

        else:
            rospy.loginfo("Failure in reaching given position")

            # Try new positioning
            self.ar_positioned = False

        # Send log messages
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
        rospy.sleep(3)
        return self.tf_listener.lookupTransform('/map', '/ar_marker_0', rospy.Time(0))

    def ar_in_position(self):
        return self.ar_positioned

    def is_img_centered(self):
        return self.img_centered

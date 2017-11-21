#!/usr/bin/env python

# ROS and OpenCV packages
from __future__ import division
import cv2
import rospy
import sys
import tf
import time
import os
from heapq import nlargest

# Topics messages types
from geometry_msgs.msg import Twist, Vector3
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Helper functions
from helpers import processing

"""
    The class functionality is to
    detect where the pose of the
    image is and take a snaphot of
    it.
"""
class Detect:

    # Constructor
    def __init__(self):

        # OpenCV interface instance
        self.bridge = CvBridge()

        # RGB image subscriber
        self.cv_image_listener = rospy.Subscriber('camera/rgb/image_raw', Image, self.convert)

        # Velocity publisher
        self.velocity_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size = 10)

        # AR tracker listener
        self.ar_tracker = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.marker)

        # Velocity variable
        self.velocity = Twist()

        # Current image
        self.cv_image = None

        # Self centered
        self.centered = False

        # Image frame
        self.x = 640
        self.y = 480

    # Marker callaback
    def marker(self, data):

        # Check if marker detected
        if not data.markers:
            # Spin robot as to find a marker
            self.velocity.angular.z = 0.2
            print("Finding markers ...")

        elif data.markers and not self.centered:
            # Center image and stop spinning
            self.velocity.angular.z = 0
            self.center(self.cv_image)

        else:
            self.velocity.angular.z = 0
            print("Processed finished")

        self.velocity_pub.publish(self.velocity)

    # Converts image into MAT format
    def convert(self, data):

        try:
            # RGB raw image to OpenCV bgr MAT format
            self.cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        except Exception as CvBridgeError:
            print('Error during image conversion: ', CvBridgeError)

    def center(self, image):

        # Show image
        cv2.namedWindow('Original Image')
        cv2.imshow('Original Image', image)
        cv2.waitKey(5)

        print("Centering in process")

        # Apply convolutional kernel to smooth image
        blurred = blur(image)

        # Convert image to greyscale
        grey = greyscale(blurred)

        # Show image
        cv2.namedWindow('Greyscale Image')
        cv2.imshow('Greyscale Image', grey)
        cv2.waitKey(5)

        # Apply adaptive thresholding
        thresh = threshold(grey)

        # Show image
        cv2.namedWindow('Thresholded Image')
        cv2.imshow('Thresholded Image', thresh)
        cv2.waitKey(5)

        # Closing on the thresholded image
        closing = morph(thresh)

        # Show image
        cv2.namedWindow('Closing Image')
        cv2.imshow('Closing Image', closing)
        cv2.waitKey(5)

        # Canny edge detector
        edges = canny(closing)

        # Show image
        cv2.namedWindow('Canny')
        cv2.imshow('Canny', edges)
        cv2.waitKey(5)

        # Find the contours
        (contours, _) = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Compute areas for contours found
        cnt_areas = [cv2.contourArea(contours[n]) for n in range(len(contours))]

        # Get biggest contour index
        maxCnt_index = cnt_areas.index(max(cnt_areas))

        cv2.drawContours(image, contours, maxCnt_index, (0,255,0), 3)

        # Show image
        cv2.namedWindow('Contour')
        cv2.imshow('Contour', image)
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

            # Take a snapshot of the centered image
            cv2.imwrite(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'data/detections/image.png')), image)

            # tf listener
            listener = tf.TransformListener()
            time.sleep(3)

            # Write image position
            try:
                file = open(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'data/poses.txt')), "w")
                file.write(str(listener.lookupTransform('/map', '/ar_marker_0', rospy.Time(0))[0]))
                file.close()

            except Exception as e:
                print("Error while writing image pose: ", e)

            self.centered = True

        # Publish velocity
        self.velocity_pub.publish(self.velocity)

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
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError
from ar_track_alvar_msgs.msg import AlvarMarkers

# Routines
from helpers import blur, greyscale, threshold, morph, canny, GoToPose, Recognition

class Position:

    # Constructor
    def __init__(self):

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
        self.ar_positioning = False
        self.img_processing = False

        # Rate
        self.rate = rospy.Rate(10)

        

    # Marker callaback
    def get_pose(self, data):

        if data.markers and not self.ar_positioning:

            # Stop marker data flow
            self.ar_positioning = True

            # Get ar marker tranformation matrix (respect to the map)
            time.sleep(3)
            (trans, rotation) = self.tf_listener.lookupTransform('/map', '/ar_marker_0', rospy.Time(0))

            # Build rotation matrix
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

            if success:

                rospy.loginfo("Given map position reached")

                # Start image processing
                self.img_processing = True

            else:
                rospy.loginfo("Failure in reaching given position")

                # Try new positioning
                self.ar_positioning = False

            # Sleep to give the last log messages time to be sent
            rospy.sleep(1)

    # Converts image into MAT format
    def recognise(self, data):

        if self.img_processing and not self.img_centered:

            try:
                # RGB raw image to OpenCV bgr MAT format
                image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

                # Apply convolutional kernel to smooth image
                blurred = blur(image)

                # Convert image to greyscale
                grey = greyscale(blurred)

                # Apply adaptive thresholding
                thresh = threshold(grey)

                # Show image
                cv2.namedWindow('Thresholded Image')
                cv2.imshow('Thresholded Image', thresh)
                cv2.waitKey(5)

                # Canny edge detector
                edges = canny(thresh)

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

                    # else:



                # Publish velocity
                self.velocity_pub.publish(self.velocity)

            except Exception as CvBridgeError:
                print('Error during image conversion: ', CvBridgeError)

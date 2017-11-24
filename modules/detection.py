#!/usr/bin/env python

# ROS and OpenCV packages
from __future__ import division
import cv2
import rospy
import time
import math
import tf
import os
import numpy as np

# Topics messages types
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError
from ar_track_alvar_msgs.msg import AlvarMarkers

# Routines
from helpers import blur, greyscale, threshold, morph, canny, GoToPose, Recognition

"""
    The class functionality is to
    detect where the pose of the
    image is and take a snaphot of
    it.
"""
class Detection:

    # Constructor
    def __init__(self):

        # OpenCV interface
        self.bridge = CvBridge()

        # Base velocity
        self.velocity = Twist()

        # Flags
        self.positioned = False
        self.img_recognition = False

        # Image frame
        self.x = 640
        self.y = 480

        # Rate
        self.rate = rospy.Rate(10)

        # AR marker listener
        self.ar_tracker = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.get_pose)

        # RGB image subscriber
        self.image_raw = rospy.Subscriber('camera/rgb/image_raw', Image, self.recognise)

        # Velocity publisher
        self.velocity_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size = 10)

    # Converts image into MAT format
    def recognise(self, data):

        if self.img_recognition:

            try:
                # RGB raw image to OpenCV bgr MAT format
                image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

                # Show image
                cv2.namedWindow('Real Time')
                cv2.imshow('Real Time', image)
                cv2.waitKey(5)

                print("Centering in process")

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

                    # # Recognise image
                    # tbr = cv2.imread(os.path.join(os.path.dirname( __file__ ), '..', 'data/detections/image.png'))
                    # if Recognition().checkImage(tbr):
                    #     print("Found: ", Recognition().checkImage(tbr))
                    #     self.img_recognition = False
                    #
                    # else:
                    #     print("Found nothing.")

                    # Take a snapshot of the centered image
                    cv2.imwrite(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'data/detections/image.png')), image)

                    # # tf listener
                    # listener = tf.TransformListener()
                    # time.sleep(3)

                    # # Write image position
                    # try:
                    #     file = open(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'data/poses.txt')), "w")
                    #     file.write(str(listener.lookupTransform('/map', '/ar_marker_0', rospy.Time(0))[0]))
                    #     file.close()
                    #
                    # except Exception as e:
                    #     print("Error while writing image pose: ", e)

                # Publish velocity
                self.velocity_pub.publish(self.velocity)

            except Exception as CvBridgeError:
                print('Error during image conversion: ', CvBridgeError)

    # Marker callaback
    def get_pose(self, data):

        # Check if marker detected
        # if not data.markers:
        #     # Spin robot as to find a marker
        #     self.velocity.angular.z = 0.2
        #     print("Finding markers ...")
        #
        # elif data.markers and not self.centered:
        #     # Center image and stop spinning
        #     self.velocity.angular.z = 0
        #     self.center(self.cv_image)
        #
        # else:
        #     self.velocity.angular.z = 0
        #     print("Processed finished")
        #
        # self.velocity_pub.publish(self.velocity)

        if data.markers and not self.positioned:

            # Stop marker data flow
            self.positioned = True

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
            pose = trans[:2] + direction * 0.4

            # Get desired robot rotation
            theta = math.atan2(pose[1], pose[0])

            print("Pose: ", pose)
            print("Theta: ", theta)

            # Build desired pose
            x, y, z = pose[0], pose[1], theta
            position = {'x': x, 'y' : y}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

            print("Position: ", position)
            print("Quaternion: ", quaternion)

            # Send robot to pose
            rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
            success = GoToPose().goto(position, quaternion)

            if success:

                # Stop marker positioning
                self.positioned = True

                # # Rotate robot for 60 degrees right
                # self.velocity.angular.z = -math.radians(70)
                # start_time = time.time()
                # for x in range(30):
                #     print("rotating")
                #     if (time.time() - start_time < 1):
                #         self.velocity_pub.publish(self.velocity)
                #     self.rate.sleep()

                rospy.loginfo("Given map position reached")

                # Start image recognition
                self.img_recognition = True

            else:
                rospy.loginfo("Failure in reaching given position")

                # Try new positioning
                self.positioned = False

            # Sleep to give the last log messages time to be sent
            rospy.sleep(1)

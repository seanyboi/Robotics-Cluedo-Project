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

# Messages
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError

# Routines
from gotopose import GoToPose
from helpers import toMAT, img_processing, get_contour

class Position:

    def __init__(self):
        """ Class constructor """

        # Camera size
        self.x = 640
        self.y = 480

        # Object instances
        self.gtp = GoToPose()
        self.velocity = Twist()
        self.bridge = CvBridge()
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
        (trans, rotation) = self.get_ar_transform()

        # Get rotation matrix
        matrix = self.tf_listener.fromTranslationRotation(trans, rotation)

        # Get z column in the matrix
        direction = matrix[:3 , 2]

        # Compute desired point (in front of ar_marker)
        pose = trans + direction * 0.4

        # Flip direction
        theta_direction = direction * -1

        # Normalise theta_direction
        theta_direction_norm = theta_direction / (theta_direction[0] ** 2 + theta_direction[1] ** 2) ** 0.5

        # Get desired robot rotation
        theta = math.atan2(theta_direction_norm[1], theta_direction_norm[0])

        # Send robot to pose
        success = self.gtp.goto(pose[0], pose[1], theta)

        if success and self.get_ar_transform()[0]:

            rospy.loginfo("Given map position reached")

            # Start image centering
            self.ar_positioned = True

        elif success and not get_ar_transform()[0]:

            rospy.loginfo("Given map position reached but AR marker offset... Starting recover procedure")

            # Send log messages
            rospy.sleep(1)

            # Rotate robot and check for AR marker
            while not rospy.is_shutdown():

                # Rotation
                self.velocity.linear.x = 0
                self.velocity.angular.z = getRadians(15)

                # Rotate
                for x in range(30):
                    pub.publish(desired_velocity)
                    self.rate.sleep()

                if get_ar_transform()[0]:
                    break

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

        # RGB raw image to OpenCV bgr MAT format
        cv_image = toMAT(raw_image)

        # Image processing
        processed_img = img_processing(cv_image)

        # Get contours and max index
        contours, maxCnt_index = get_contour(processed_img)

        # Draw contours
        cv2.drawContours(cv_image, contours, maxCnt_index, (0,255,0), 3)

        # Contour pose (for centering purposes)
        M = cv2.moments(contours[maxCnt_index])
        max_cnt_x = int(M['m10']/M['m00'])
        max_cnt_y = int(M['m01']/M['m00'])

        # Centre image within the frame
        if self.x - max_cnt_x > 340:
            self.velocity.angular.z = 0.2

        elif self.x - max_cnt_x < 280:
            self.velocity.angular.z = -0.2

        else:
            # Stop rotation (centering completed)
            self.velocity.angular.z = 0
            self.img_centered = True

        # Publish velocity
        self.velocity_pub.publish(self.velocity)

        # Return image for recognition
        return cv_image

    def get_ar_transform(self):
        rospy.sleep(3)
        return self.tf_listener.lookupTransform('/map', '/ar_marker_0', rospy.Time(0))

    def ar_in_position(self):
        return self.ar_positioned

    def is_img_centered(self):
        return self.img_centered

    def reset_ar_flag(self):
        self.ar_positioned = False

    def reset_center_flag(self):
        self.img_centered = False

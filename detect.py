#!/usr/bin/env python

from __future__ import division
import cv2
import numpy as np
import rospy
import roslib
import sys
import tf
import time

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Detect():

    # Constructor
    def __init__(self):

        # OpenCV interface instance
        self.bridge = CvBridge()

        # RGB image subscriber
        self.image_listener = rospy.Subscriber('camera/rgb/image_raw', Image, self.convert)

        # Velocity publisher
        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist)

        # Velocity variable
        self.velocity = Twist()

        # Current image
        self.image = None

        # Transform listener
        self.listener = tf.TransformListener()

        # Image frame
        self.x = 640
        self.y = 480

    # Converts image into MAT format
    def convert(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            self.image = cv2.blur(cv_image, (5,5))

        except Exception as CvBridgeError:
            print('Error during image conversion: ', CvBridgeError)

    # Checks for images (AR Marker)
    def spin(self):

        if !self.listener.lookupTransform('/map', '/ar_marker_0', rospy.Time(0))[0]:
            self.velocity.z = 0.2

        else:
            self.center()

    def center(self):

        # Find contours in the image
        imgray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(imgray,127,255,0)
        im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        cnt_areas = [cv2.contourArea(contours[n]) for n in range(len(contours))]
        maxCnt = max(cnt_areas)

        cv2.drawContours(self.image, [maxCnt], 0, (0,255,0), 3)

        # Get center pose of the contour
        M = cv2.moments(maxCnt)
        cnt_x = int(M['m10']/M['m00'])

        # Centre image
        if self.x - cx > 330:
            velocity.angular.z = 0.3
        elif self.x - cx < 300:
            velocity.angular.z = -0.3

        # Take a snapshot of the centered image
        cv2.imwrite('./detections/image.jpeg', self.image)

        # Write image position
        try:
            file = open(“poses.txt”,”w”)
            file.write(str(self.listener.lookupTransform('/map', '/ar_marker_0', rospy.Time(0))[0]))
            file.close()

        except Exception as e:
            print("Error while writing image pose")

def main(args):

    # Initialise node
    rospy.init_node('image_converted', anonymous=True)
    rospy.init_node('tf_example',anonymous=True)

    try:
        # Asking tf to find the transform of child_frame in parent_frame.
        rospy.spin()
        
    except (KeyboardInterrupt, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print('Error during main execution' + e)

        cv2.destroyAllWindows()

        # Check if the node is executing in the main path
        if __name__ == '__main__':
            main(sys.argv)

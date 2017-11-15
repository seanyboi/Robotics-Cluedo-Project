#!/usr/bin/env python

# ROS and OpenCV packages
from __future__ import division
import cv2
import numpy as np
import rospy
import roslib
import sys
import tf
import time

# Topics messages types
from geometry_msgs.msg import Twist, Vector3
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

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

        # AR tracker listener
        self.ar_tracker = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.marker)

        # RGB image subscriber
        self.cv_image_listener = rospy.Subscriber('camera/rgb/image_raw', Image, self.convert)

        # Velocity publisher
        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size = 10)

        # Velocity variable
        self.velocity = Twist()

        # Current image
        self.cv_image = None

        # Transform listener
        self.listener = tf.TransformListener()

        # Image frame
        self.x = 640
        self.y = 480

    # Marker callaback
    def marker(self, data):

        # Check if marker detected
        if data.markers:
            # Center image
            self.center(self.cv_image)

        else:
            # Spin robot as to find an image
            self.velocity.z = 0.2

    # Converts image into MAT format
    def convert(self, data):

        try:
            # RGB raw image to OpenCV bgr MAT format
            self.cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # Show image
            cv2.namedWindow('self.cv_image')
            cv2.imshow('self.cv_image', self.cv_image)
            cv2.waitKey(5)

        except Exception as CvBridgeError:
            print('Error during image conversion: ', CvBridgeError)

    # Checks for images (AR Marker)
    def spin(self):

        try:

            if not self.listener.lookupTransform('/map', '/ar_marker_0', rospy.Time(0))[0]:
                self.velocity.z = 0.2

            else:
                self.center()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("Error during marker transformation: ", e)

    def center(self, image):

        # Apply convolutional kernel to smooth image
        blurred_image = cv2.blur(image, (5,5))

        # Convert image to greyscale
        grey_scale_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2GRAY)

        # Apply threshold function (get rid of background)
        thresh = cv2.threshold(grey_scale_image, 90, 255, 0)[1]

        # Closing on the thresholded image
        closing = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, (5,5))

        # Canny edge detector
        edges = cv2.Canny(closing,100,200)

        # Show image
        cv2.namedWindow('Closed')
        cv2.imshow('Closed', edges)
        cv2.waitKey(5)

        # Find the contours
        contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Compute areas for contours found
        cnt_areas = [cv2.contourArea(contours[n]) for n in range(len(contours))]

        # Get biggest contour
        maxCnt = max(cnt_areas)
        maxCnt_index = cnt_areas.index(maxCnt)

        # Draw the contour
        cv2.drawContours(image, contours, maxCnt_index, (0,255,0), 3)

        # Show image
        cv2.namedWindow('Contour')
        cv2.imshow('Contour', image)
        cv2.waitKey(5)

        # Contour pose (for centering purposes)
        M = cv2.moments(maxCnt)
        max_cnt_x = int(M['m10']/M['m00'])

        # Centre image within the frame
        if self.x - max_cnt_x > 330:
            self.velocity.angular.z = 0.3

        elif self.x - max_cnt_x < 300:
            self.velocity.angular.z = -0.3

        else:

            # Take a snapshot of the centered image
            cv2.imwrite('./data/detections/image.jpeg', self.cv_image)

            # Write image position
            try:
                file = open("poses.txt", "w")
                file.write(str(self.listener.lookupTransform('/map', '/ar_marker_0', rospy.Time(0))[0]))
                file.close()

            except Exception as e:
                print("Error while writing image pose")

def main(args):

    # Initialise node
    rospy.init_node('detection', anonymous=True)

    detector = Detect()

    try:
        # Asking tf to find the transform of child_frame in parent_frame.
        rospy.spin()

    except (KeyboardInterrupt, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print('Error during main execution' + e)

        cv2.destroyAllWindows()

# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)

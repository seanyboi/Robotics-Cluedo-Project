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



if __name__ == '__main__':
    rospy.init_node('tf_example',anonymous=True)

    listener = tf.TransformListener()

    try:
        # Asking tf to find the transform of child_frame in parent_frame.
        (trans,rot) = listener.lookupTransform('/map', '/ar_marker_0', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass

    print trans
    print rot

class Detect():

	def __init__(self):
		self.bridge = CvBridge()
		self.image_listener = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)
		self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist)

        self.velocity = Twist()

		self.x = 640
		self.y = 480

    def spin(self):

        if !listener.lookupTransform('/map', '/ar_marker_0', rospy.Time(0))[0]:
            self.velocity.z = 0.2

        else:
            self.center()

    def center(self):

        # Convert ROS image type to MAT format
        cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        cv2.namedWindow('Original')
        cv2.imshow('Original', cv_image)
        cv2.waitKey(5)




	def callback(self, data):
		try:
			# Convert ROS image type to MAT format
			cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
			cv2.namedWindow('Original')
			cv2.imshow('Original', cv_image)
			cv2.waitKey(5)

			# Blur image (to reduce noise)
			blur = cv2.blur(cv_image, (5,5))
			cv2.namedWindow('Blurred')
			cv2.imshow('Blurred', blur)
			cv2.waitKey(5)

			# Change rgb format to hsv
			hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

			# Green mask
			green_mask = cv2.inRange(hsv, self.hsv_lower_green, self.hsv_upper_green)

			# Blue mask
			# blue_mask = cv2.inRange(hsv, self.hsv_lower_blue, self.hsv_upper_blue)

			# Red mask
			# red_mask = cv2.inRange(hsv, self.hsv_lower_red, self.hsv_upper_red)

			# Intermediate mask
			# int_mask = cv2.bitwise_or(green_mask, blue_mask)

			# Final mask
			# mask = cv2.bitwise_or(int_mask, red_mask)

			# Find contours
			contours = cv2.findContours(green_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]

			# Apply mask to original image
			res = cv2.bitwise_and(blur, blur, mask=green_mask)

			# Velocity
			velocity = Twist()

			if len(contours) == 0:
				velocity.angular.z = 0.3
				self.pub.publish(velocity)

			for cnt in contours:

				# Get contour area
				cnt_area = cv2.contourArea(cnt)
				print("Contour area: ", cnt_area)

				if cnt_area > 800:

					# Get contour centroids
					M = cv2.moments(cnt)
					cx = int(M['m10']/M['m00'])
					cy = int(M['m01']/M['m00'])
					radius = int((cnt_area/3.14)**0.5)

					# Draw contours
					cv2.circle(res, (cx, cy), radius, (255,255,255), 4)

					# Change color flag
					# redpixelpoints = cv2.findNonZero(red_mask)
					# bluepixelpoints = cv2.findNonZero(blue_mask)
					greenpixelpoints = cv2.findNonZero(green_mask)

					# self.red_flag = 1 if redpixelpoints != None  else 0
					# self.blue_flag = 1 if bluepixelpoints != None else 0
					self.green_flag = 1 if greenpixelpoints != None else 0
					print("[red, green, blue]: ", [self.red_flag, self.green_flag, self.blue_flag])

					# If red keep tracking
					# if self.green_flag == 1 and self.red_flag == 0:
					if self.green_flag == 1:

						print("Green area: ", cnt_area)

						# Center red
						if self.x - cx > 330:
							velocity.angular.z = 0.3
						elif self.x - cx < 300:
							velocity.angular.z = -0.3
						else:
							velocity.angular.z = 0

							if cnt_area > 70000:
								velocity.linear.x = -0.2

							if cnt_area < 20000:
								velocity.linear.x = 0.2

					elif self.green_flag == 1 and self.red_flag == 1:
						velocity.linear.x = 0
						velocity.angular.z = 0

					else:
						velocity.angular.z = 0.2

					self.pub.publish(velocity)

		except Exception as CvBridgeError:
			print('Error during image conversion: ', CvBridgeError)

		# Display result
		cv2.namedWindow('Camera_Feed')
		cv2.imshow('Camera_Feed', res)
		cv2.waitKey(5)

def main(args):
	# Initialise node
	rospy.init_node('image_converted', anonymous=True)

	# colourIdentifier instance
	cI = colourIdentifier()

	# Spin the node
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print('Shutting down node')

	cv2.destroyAllWindows()

# Check if the node is executing in the main path
if __name__ == '__main__':
	main(sys.argv)

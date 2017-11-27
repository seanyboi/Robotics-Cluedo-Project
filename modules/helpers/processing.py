#!/usr/bin/env python

# ROS and OpenCV packages
from __future__ import division
import cv2

from cv_bridge import CvBridge, CvBridgeError

# Blur the image
def blur(img):
    return cv2.medianBlur(img, 5)

# Convert to greyscale
def greyscale(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Adaptive thresholding
def threshold(img):
    return cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 17, 22)

# Morphological closing
def morph(img):
    return cv2.morphologyEx(img, cv2.MORPH_OPEN, (5,5))

# Canny edge detector
def canny(img):
    return cv2.Canny(img, 100, 200)

# Raw to OpenCV conversion
def toMAT(raw_image):
    try:
        # RGB raw image to OpenCV bgr MAT format
        cv_image = CvBridge().imgmsg_to_cv2(raw_image, 'bgr8')

        # Return OpenCV version
        return cv_image

    except Exception as CvBridgeError:
        print('Error during image conversion: ', CvBridgeError)

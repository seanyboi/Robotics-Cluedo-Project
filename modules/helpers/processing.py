#!/usr/bin/env python

# ROS and OpenCV packages
from __future__ import division
import cv2

from cv_bridge import CvBridge, CvBridgeError

# Blur the image
def img_processing(img):
    # Blurring
    img_blur = cv2.medianBlur(img, 5)

    # Greyscale conversion
    img_greyscale = cv2.cvtColor(img_blur, cv2.COLOR_BGR2GRAY)

    # Adaptive thresholding
    img_thresholded = cv2.adaptiveThreshold(img_greyscale,
                                            255,
                                            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                            cv2.THRESH_BINARY,
                                            17,
                                            22)

    # Canny edge detector
    img_canny = cv2.Canny(img, 100, 200)

    # Return processed image
    return img_canny

# Obtain biggest contour
def get_contour(img):
    # Find the contours
    (contours, _) = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Compute areas for contours found
    cnt_areas = [cv2.contourArea(contours[n]) for n in range(len(contours))]

    # Get biggest contour index
    maxCnt_index = cnt_areas.index(max(cnt_areas))

    # return contours and max index
    return contours, maxCnt_index

# Raw to OpenCV conversion
def toMAT(raw_image):
    try:
        # RGB raw image to OpenCV bgr MAT format
        cv_image = CvBridge().imgmsg_to_cv2(raw_image, 'bgr8')

        # Return OpenCV version
        return cv_image

    except Exception as CvBridgeError:
        print('Error during image conversion: ', CvBridgeError)

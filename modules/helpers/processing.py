#!/usr/bin/env python

# ROS and OpenCV packages
from __future__ import division
import os
import cv2
import numpy as np

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

# Template matching for image boundary
def temp_matching(template, cv_image):

    template = cv2.imread(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', '..', 'data/images/%(template)s.png' % locals())))
    template = cv2.resize(template, None, fx = 0.3, fy = 0.3, interpolation = cv2.INTER_CUBIC)
    t, w, h = template.shape[::-1]
    print("Shape: ", w, h, t)

    method = 'cv2.TM_CCOEFF_NORMED'

    # Apply template Matching
    res = cv2.matchTemplate(cv_image,
                            template,
                            eval(method))
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

    # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
    if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
        top_left = min_loc
    else:
        top_left = max_loc

    bottom_right = (top_left[0] + w, top_left[1] + h)
    cv2.rectangle(cv_image, top_left, bottom_right, 255, 2)
    return cv_image

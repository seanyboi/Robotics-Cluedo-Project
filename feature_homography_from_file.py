#!/usr/bin/env python

'''
Feature homography
==================

Example of using features2d framework for video homography matching.
ORB features and FLANN matcher are used. The actual tracking is implemented by
PlaneTracker class in plane_tracker.py

Inspired by http://www.youtube.com/watch?v=-ZNYoL8rzPY

video: http://www.youtube.com/watch?v=FirtmYcC0Vc

Usage
-----
feature_homography_from_file1.py [<video source>]

'''

# Python 2/3 compatibility
from __future__ import print_function

import numpy as np
import cv2

# local modules
import video
import common
from common import getsize, draw_keypoints
from plane_tracker import PlaneTracker


class App:

    def __init__(self, src):

        self.cap = video.create_capture(src)
        self.frame = None
        self.paused = False
        self.tracker = PlaneTracker()

		#Add template images to tracker
        frame = cv2.imread('mustard.png')
        rect = (0, 0, frame.shape[1], frame.shape[0])
        self.tracker.add_target(frame.copy(), rect, None)

        cv2.namedWindow('plane')

    def run(self):

        while True:
            playing = not self.paused

            if playing or self.frame is None:

                ret, frame = self.cap.read()
                if not ret:
                    break

                self.frame = frame.copy()

                w, h = getsize(self.frame)
                vis = np.zeros((h, w, 3), np.uint8)

                vis[:h,:w] = self.frame

                tracked = self.tracker.track(self.frame)

                if len(tracked) > 0:

                        for tracked_ob in tracked:

                            print ('Found ' + tracked_ob.target.data)
                            # Calculate Homography
                            h, status = cv2.findHomography(tracked_ob.p0, tracked_ob.p1)

                cv2.imshow('plane', vis)

                ch = cv2.waitKey(1)
                if ch == 27:
                    break

if __name__ == '__main__':
    print(__doc__)

    import sys
    try:
        video_src = sys.argv[1]

    except:
        video_src = 0

    App(video_src).run()

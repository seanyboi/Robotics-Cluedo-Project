#!/usr/bin/env python

"""
    Main.

    Listens to in-coming data.
    Handles the logic by calling the necessary
    methods, including navigation and vision ones.
"""

# Modules
from __future__ import print_function
from __future__ import division
import rospy
import sys
import cv2
import os
import tf
import random
import math

# Ar-marker message data type
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from ar_track_alvar_msgs.msg import AlvarMarkers

# Modules
from kobuki_msgs.msg import BumperEvent
from modules import Navigation, Position, Recognition, GoToPose

class RoboticsCluedo:

    def __init__(self):
        """ Class constructor """

        # Counter
        self.counter = 0

        # Detections array
        self.poses = []
        self.detections = []

        # Flag
        self.process = True
        self.initialised = False

        # Ar-marker and Scan data
        self.img_raw = None
        self.ar_data = None
        self.scan_data = None
        self.bumper_data = None

        # Object instances
        self.gtp = GoToPose()
        self.pst = Position()
        self.nvg = Navigation()
        self.rcg = Recognition()

        # Marker & Image subscribers
        self.sub = rospy.Subscriber('scan', LaserScan, self.set_scan_data)
        self.ar_tracker = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.set_ar_data)
        self.image_raw = rospy.Subscriber('camera/rgb/image_raw', Image, self.set_raw_image)

    def run(self, x, y):
        """
            The routine instructs the robot to position
            itself in a precise location in the map and
            starts the logic afterwards.

            Arguments:
                param1: The x coordinate in the map
                param2: The y coordinate in the map
        """
        # Send robot to pose
        success = self.gtp.goto(x, y, 0)

        if success:
            # Start logic
            print("Starting the logic....")
            self.logic()
        else:
            print("Failed to go to inital pose... Rotation and trying again")
            # self.nvg.rotate(180)
            self.run(x, y)

    def logic(self):
        """
            Robot's logic handling decisions for when to navigate
            and when to run the computer vision modules, such
            as detection and recognition.

            Arguments:
                param1: The incoming data of the ar-marker
        """
        while len(self.detections) < 2:

            # Get latest data
            data = self.get_ar_data()

            # Run vision logic
            if bool(data) and self.process and not self.already_visited():

                while not self.rcg.is_recognised() and self.counter < 3:

                    # Position in front of the AR
                    if not self.pst.ar_in_position():
                        rospy.loginfo("AR positioning...")
                        self.pst.toAR()
                        self.counter += 1

                    # Center in front of the image
                    elif self.pst.ar_in_position and not self.pst.img_centered:
                        rospy.loginfo("Image centering...")
                        self.pst.center_image(self.get_raw_image())
                        self.counter = 0

                    # Recognise image
                    elif self.pst.ar_in_position and self.pst.img_centered:
                        rospy.loginfo("Recognition...")
                        # Check if image recognised is already done
                        res = self.rcg.recognise(self.get_raw_image())
                        print("Res from rcg: ", res)
                        if res and res[0] not in self.detections:
                            self.detections.append(res[0])
                            self.pose_and_snapshot(res[0], res[1])
                        else:
                            # Break the loop and carry on search
                            print("Tried 3 times detecting not working, moving on...")
                            break

                # Start new search
                self.process = False
                self.pst.reset_ar_flag()
                self.rcg.reset_rec_flag()
                self.pst.reset_center_flag()

            elif bool(data) and not self.process:
                rospy.loginfo("Starting new search...")
                self.nvg.rotate(random.randint(90, 180))
                self.process = True
                self.counter = 0

            else:
                rospy.loginfo("Robot is scanning the room")
                self.nvg.navigate(self.get_scan_data())
                self.counter = 0

        # Acknowledge task completition
        rospy.loginfo("MISSION ACCOMPLISHED, HOUSTON !")

    def pose_and_snapshot(self, res, image):
        """
            Writes the pose of the image
            to the poses.txt file and saves
            the snapshot of the image in the
            data folder.

            Arguments:
                param1: OpenCV type image
                param2: Name of the detected image
        """
        try:
            # Save image under detections
            cv2.imwrite(os.path.abspath(os.path.join(os.path.dirname( __file__ ), 'data/detections/%(res)s.png' % locals())), image)

            # Get ar pose
            (trans, _) = self.pst.get_ar_transform()

            # Write to file (position of the image)
            file = open(os.path.abspath(os.path.join(os.path.dirname( __file__ ), 'data/poses.txt')), "a")
            file.write('%(res)s: ' % locals() + str(trans))
            file.close()

            # Store ar pose
            self.poses.append(trans)

        except Exception as e:
            print("Error while writing image pose: ", e)

    def already_visited(self):

        try:
            # Get ar-marker trans
            trans = self.pst.get_ar_transform()[0]
            print("Curr trans: ", trans)
            print("Poses len: ", len(self.poses))

            if trans and len(self.poses) > 0:

                for pose in self.poses:
                    print("Pose: ", pose)
                    res = ((self.abs(trans[0] - pose[0]) <= 0.4) and (self.abs(trans[1] - pose[1]) <= 0.4))
                    if res:
                        print("VISITED...")
                        return True
                    else:
                        print("NOT VISITED...")
                        return False

            elif trans and len(self.poses) == 0:
                print("NOT VISITED...")
                return False

            else:
                print("No valid trans...")
                return False

        except Exception as e:
            print("Error while checking previous poses... Continue search anyway")

    def abs(self, x):
        return math.fabs(x)

    def set_raw_image(self, data):
        """
            Sets the incoming camera
            data to a global variable.

            Arguments:
                param1: Raw image data
        """
        self.img_raw = data

    def set_scan_data(self, data):
        """
            Sets the incoming LaserScan
            data to a global variable.

            Arguments:
                param1: LaserScan data
        """
        self.scan_data = data

    def set_ar_data(self, data):
        """
            Sets the incoming ar-marker
            data to a global variable.

            Arguments:
                param1: Ar-marker data
        """
        self.ar_data = data

    def get_raw_image(self):
        """
            Returns the latest image data.

            Returns:
                RGB raw image
        """
        return self.img_raw

    def get_scan_data(self):
        """
            Returns the latest laserscan data.

            Returns:
                LaserScan ranges
        """
        return self.scan_data.ranges

    def get_ar_data(self):
        """
            Returns the latest ar-marker data.

            Returns:
                coordinates of the ar-marker
        """
        return self.ar_data.markers

def main(args):

    # Initialise node
    rospy.init_node('robotics_cluedo', anonymous=True)

    try:

        # Application instance
        rc = RoboticsCluedo()

        # Let data flow
        rospy.loginfo("Getting data...")
        rospy.sleep(3)

        # Run the logic
        rc.run(1.4, -3.94)

        # Spin it baby !
        rospy.spin()

    except KeyboardInterrupt as e:
        print('Error during main execution' + e)

# Execute main
if __name__ == '__main__':
    main(sys.argv)

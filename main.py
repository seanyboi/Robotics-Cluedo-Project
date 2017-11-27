#!/usr/bin/env python

"""
    Main.
"""

# Modules
from __future__ import division
import rospy
import sys

# Ar-marker message data type
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers

# Logic blocks
from modules import Navigation, Position, Recognition, GoToPose

"""
    The following class handles the robot's logic
    instructing it when to do what and is which
    circumstances.
"""
class RoboticsCluedo:

    def __init__(self):
        """ Class constructor """

        # Flags
        self.process = True
        self.initialised = False

        # Object instances
        self.gtp = GoToPose()
        self.pst = Position()
        self.nvg = Navigation()
        self.rcg = Recognition()

        # Marker & Image subscribers
        self.image_raw = rospy.Subscriber('camera/rgb/image_raw', Image, self.get_raw_image)
        self.ar_tracker = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.get_ar_data)

    def send(self, x, y):
        """
            The following routine instructs the robot
            to position itself in a precise location
            in the map.

            Arguments:
                param1: The x coordinate in the map
                param2: The y coordinate in the map

            Return:
                bool: True or False (indicating success or failure)
        """
        # TODO: run GoToPose method
        rospy.loginfo("Robot reached the given position")

        # Allow logic to be run
        self.initialised = True

    def logic(self, data):
        """
            The function handles the logic of the robot,
            deciding when to navigate by scanning the wall,
            and when to run the image recognition process.

            Arguments:
                param1: The incoming data of the ar-marker
        """

        if initialised:

            # Run vision logic
            if data.markers and self.process:

                while not self.rcg.is_recognised():

                    if not self.pst.ar_in_position():
                        self.pst.toAR()
                        rospy.loginfo("AR positioning...")

                    elif self.pst.ar_in_position() and not self.pst.img_centered():
                        self.pst.center_image(get_raw_image())
                        rospy.loginfo("Image centering...")

                    elif self.pst.ar_in_position() and self.pst.img_centered():
                        self.rcg.recognise(get_raw_image())
                        rospy.loginfo("Recognition...")

                    # Send log messages
                    rospy.sleep(1)

                self.process = False

            # Navigate the map
            else:
                # TODO: Insert navigation logic
                rospy.loginfo("Robot is scanning the room")

    def get_ar_data(self, data):
        """
            Getter function that returns
            the ar-marker incoming data.

            Arguments:
                param1: Ar-marker data

            Returns:
                Ar-marker data
        """
        return data

    def get_raw_image(self, data):
        """
            Getter function that returns
            the raw image data.

            Arguments:
                param1: Raw image data

            Returns:
                Raw image data
        """
        return data

def main(args):

    # Initialise node
    rospy.init_node('robotics_cluedo', anonymous=True)

    # Flag
    firstTime = True

    # Application instance
    rc = RoboticsCluedo()

    try:

        # Center the robot
        # in the map
        if firstTime:
            rc.send(x, y)
            firstTime = False

        # Instruct the robot
        # on what to do
        else:
            rc.logic()

        # Spin it baby !
        rospy.spin()

    except KeyboardInterrupt as e:
        print('Error during main execution' + e)

# Execute main
if __name__ == '__main__':
    main(sys.argv)

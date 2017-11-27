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

        # Ar-marker and Raw image data
        self.img_raw = None

        # Object instances
        self.gtp = GoToPose()
        self.pst = Position()
        self.nvg = Navigation()
        self.rcg = Recognition()

        # Marker & Image subscribers
        self.image_raw = rospy.Subscriber('camera/rgb/image_raw', Image, self.set_raw_image)
        self.ar_tracker = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.logic)

    def run(self):
        """
            The following routine instructs the robot
            to position itself in a precise location
            in the map and starts the logic afterwards.

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

    def logic(self,data):
        """
            The function handles the logic of the robot,
            deciding when to navigate by scanning the wall,
            and when to run the image recognition process.

            Arguments:
                param1: The incoming data of the ar-marker
        """
        print("Logic is running")

        if self.initialised:

            # Run vision logic
            if data.markers and self.process:

                rospy.loginfo("Processing in progress...")

                while not self.rcg.is_recognised():

                    if not self.pst.ar_in_position():
                        rospy.loginfo("AR positioning...")
                        self.pst.toAR()

                    elif self.pst.ar_in_position and not self.pst.img_centered:
                        rospy.loginfo("Image centering...")
                        self.pst.center_image(self.get_raw_image())

                    elif self.pst.ar_in_position and self.pst.img_centered:
                        rospy.loginfo("Recognition...")
                        self.rcg.recognise(self.get_raw_image())

                    # Send log messages
                    rospy.sleep(1)

                self.process = False

            # Navigate the map
            else:
                # TODO: Insert navigation logic
                rospy.loginfo("Robot is scanning the room")

    def set_raw_image(self, data):
        """
            Setter function.

            Arguments:
                param1: Raw image data
        """
        self.img_raw = data

    def get_raw_image(self):
        """
            Setter function.

            Arguments:
                param1: Raw image data
        """
        return self.img_raw

def main(args):

    # Initialise node
    rospy.init_node('robotics_cluedo', anonymous=True)

    # Application instance
    rc = RoboticsCluedo()

    try:

        # Run the game
        rc.run()

        # Spin it baby !
        rospy.spin()

    except KeyboardInterrupt as e:
        print('Error during main execution' + e)

# Execute main
if __name__ == '__main__':
    main(sys.argv)

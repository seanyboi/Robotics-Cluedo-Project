#!/usr/bin/env python

"""
    Main.
"""

# Modules
from __future__ import division
import rospy
import sys

# Ar-marker message data type
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
        self.gotopose = GoToPose()
        self.position = Position()
        self.navigation = Navigation()
        self.recognition = Recognition()

        # Marker & Image subscriber, Velocity publisher
        self.image_raw = rospy.Subscriber('camera/rgb/image_raw', Image, self.recognise)
        self.ar_tracker = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.get_pose)
        self.velocity_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size = 10)

    def center(self):
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
        rospy.loginfo("Robot reached the center of the room")

        # Allow logic to be run
        initialised = True

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
                # TODO: Insert vision logic
                rospy.loginfo("Robot is running vision logic")

            # Navigate the map
            else:
                # TODO: Insert navigation logic
                rospy.loginfo("Robot is scanning the room")

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
            rc.center()
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

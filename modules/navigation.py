#!/usr/bin/env python

"""
    Navigation.

    The class handles the navigation using
    laserscans (to overcome obstacles). M-
    oreover the navigation logic handles the
    bumping as well in case the robot hits
    the walls of obstacles
"""

# ROS and OpenCV packages
from __future__ import division
import rospy
import time
import math
import random

# Mobile base velocity message type
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist, Vector3

class Navigation:

    def __init__(self):
        """ Class constructor """

        self.mid = 0
        self.left = 0
        self.right = 0

        self.bump_state = None
        self.move_cmd = Twist()

        self.rate = rospy.Rate(10)

        self.threshold = 1

        rospy.on_shutdown(self.shutdown)

        self.side = True

        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size = 50)
        self.bumper = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.set_bumper_data)

    def navigate(self, ranges):
        """
            Navigation logic based on LaserScan.

            Arguments:
                LaserScan data ranges
        """
        if self.get_bumper_state() is None:

            # Check if min threshold hit (laser scan)
            if all(math.isnan(float(str(item))) for item in ranges):
                self.rotate(self.get_angle())

            else:

                for i in range (len(ranges)):

                    if ranges[i] < self.threshold:
                        if i < 213:
                            # rospy.loginfo("Right area")
                            self.right += 1

                        elif i > 213 and i < 426:
                            # rospy.loginfo("Middle area")
                            self.mid += 1

                        elif i > 426:
                            # rospy.loginfo("Left area")
                            self.left += 1

                rospy.loginfo("Points: \n left %d, mid %d, right %d", self.left, self.mid, self.right)

                if (self.right > self.left) and self.mid > 0:
                    rospy.loginfo("Going left")
                    self.move_cmd.angular.z= 0.6
                    self.move_cmd.linear.x= 0.1
                    self.cmd_vel.publish(self.move_cmd)

                elif (self.right > 40 and self.left > 40 and self.mid > 0):
                    rospy.loginfo("Corner")
                    self.rotate(random.randint(110, 180))

                elif (self.right > self.left) and self.mid == 0 and self.right > 40:
                    rospy.loginfo("Going left")
                    self.move_cmd.angular.z= 0.4
                    self.move_cmd.linear.x= 0.1
                    self.cmd_vel.publish(self.move_cmd)

                elif (self.left >= self.right) and self.mid > 0:
                    rospy.loginfo("Going right")
                    self.move_cmd.angular.z= -0.6
                    self.move_cmd.linear.x= 0.1
                    self.cmd_vel.publish(self.move_cmd)

                elif (self.left >= self.right) and self.mid == 0 and self.left > 40:
                    rospy.loginfo("Going right")
                    self.move_cmd.angular.z= -0.4
                    self.move_cmd.linear.x= 0.1
                    self.cmd_vel.publish(self.move_cmd)

                elif self.mid == 0 and (self.right >= 0 or self.left >= 0):
                    rospy.loginfo("Going forward")
                    print("Moving forward")
                    self.move_cmd.angular.z = 0
                    self.move_cmd.linear.x = 0.1
                    self.cmd_vel.publish(self.move_cmd)

                # Clean counters
                self.clean_counters()

        else:

            print("I am recovering from a bump... bare with me")

            # Backwards movement
            self.move_cmd.linear.x = -0.2
            self.move_cmd.angular.z = 0
            self.cmd_vel.publish(self.move_cmd)

            # Sleep for commands actuation
            rospy.sleep(1)

            # Rotate
            self.rotate(self.get_angle())

            # Clean bumper state
            self.bump_state = None

            # Clean counters
            self.clean_counters()

    def get_angle(self):
        """
            Returns a random angle
            used for bump rotation.

            Returns:
                int: angle (between 90 and 180 degrees)
        """
        return random.randint(45, 180)

    def clean_counters(self):
        """
            Clean the laser scan ranges
            values for new set of data.
        """
        self.mid = 0
        self.left = 0
        self.right = 0

    def rotate(self, deg):
        """
            Rotates the robot for a given
            amount of degrees.

            Arguments:
                param1: The degree of rotation
        """
        # Rotation
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = -math.radians(int(deg)) if self.get_choice() else math.radians(int(deg))
        self.side = not self.side

        # Time of rotation
        start_time = time.time()

        for x in range(30):
            if (time.time() - start_time < 1):
                self.cmd_vel.publish(self.move_cmd)
                self.rate.sleep()

    def get_choice(self):
        """
            Returns a random value for
            a decision making.

            Returns:
                int: 0 or 1
        """
        return random.randint(0, 1)

    def shutdown(self):
        """
            Shutdown function.
        """
        rospy.loginfo("Stopping TurtleBot")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def set_bumper_data(self, data):
        """
            Sets the bumper state to incoming
            bumper data.

            Arguments:
                param1: Bumper data
        """
        self.bump_state = data.state

    def get_bumper_state(self):
        """
            Returns the bumper state.

            Returns:
                int: bumper state
        """
        return self.bump_state

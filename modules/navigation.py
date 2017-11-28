#!/usr/bin/env python

"""
    Navigation.
"""

# ROS and OpenCV packages
from __future__ import division
import rospy
import time
import math

# Mobile base velocity message type
from geometry_msgs.msg import Twist, Vector3

class Navigation:

    def __init__(self):
        """ Class constructor """

        self.mid = 0
        self.left = 0
        self.right = 0

        self.move_cmd = Twist()

        self.rate = rospy.Rate(0.5)

        self.threshold = 0.6

        rospy.on_shutdown(self.shutdown)

        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size = 50)

    def navigate(self, ranges):
        """
            Navigation logic based on LaserScan.

            Arguments:
                LaserScan data
        """

        # Check if min threshold hit (laser scan)
        if all(math.isnan(float(str(item))) for item in ranges):


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
                self.move_cmd.angular.z= 0.5
                self.move_cmd.linear.x= 0.1
                self.cmd_vel.publish(self.move_cmd)

            if (self.left >= self.right) and self.mid > 0:
                rospy.loginfo("Going right")
                self.move_cmd.angular.z= -0.5
                self.move_cmd.linear.x= 0.1
                self.cmd_vel.publish(self.move_cmd)

            # if (self.right == self.left) and self.mid > 0:
            #     rospy.loginfo("Chosing right")
            #     self.move_cmd.angular.z= -0.6
            #     self.move_cmd.linear.x= 0.2
            #     self.cmd_vel.publish(self.move_cmd)

            if self.mid == 0 and (self.right >= 0 or self.left >= 0):
                rospy.loginfo("Going forward")
                print("Moving forward")
                self.move_cmd.angular.z = 0
                self.move_cmd.linear.x = 0.1
                self.cmd_vel.publish(self.move_cmd)

            # if self.mid == 0 and self.right == 0 and self.left == 0:
            #     rospy.loginfo("Rotating")
            #     self.move_cmd.angular.z = 0.6
            #     self.move_cmd.linear.x = 0
            #     self.cmd_vel.publish(self.move_cmd)

            self.right = 0
            self.left = 0
            self.mid = 0

    def rotate(self, deg):

        print("Rotation...")

        # Rotation
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = -math.radians(int(deg))

        # Time of rotation
        start_time = time.time()

        for x in range(100):
            if (time.time() - start_time < 2):
                self.cmd_vel.publish(self.move_cmd)
                self.rate.sleep()

    def shutdown(self):
        rospy.loginfo("Stopping TurtleBot")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import cv2
import numpy

class Robot:

    def __init__(self):
        # rrt2 is rrt data from robot 2
        self.rrt2 = rospy.Publisher('robot_2_data', String)

        # subscribe to receive robot 1 rrt data
        rospy.Subscriber('robot_1_data', String, self.callback)

    def callback(self, msg):
        rospy.loginfo("Robot 1 data received: " + str(msg))

    def robot_main(self):
        rospy.loginfo("Robot works")
        while not rospy.is_shutdown():
            self.complex_compute()
            rospy.sleep(1.0)

    def complex_compute(self):
        self.rrt2.publish("robot 2 is alive")


if __name__ == '__main__':
    rospy.init_node('robot_2')
    robot2 = Robot()
    robot2.robot_main()
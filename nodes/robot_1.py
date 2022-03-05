#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import cv2


class Robot:

    def __init__(self):
        # rrt1 is rrt data from robot 1
        self.rrt1 = rospy.Publisher('robot_1_data', String)

        # subscribe to receive robot 2 rrt data
        rospy.Subscriber('robot_2_data', String, self.callback)

    def callback(self, msg):
        rospy.loginfo("Robot 2 data received: " + str(msg))

    def robot_main(self):
        rospy.loginfo("Robot works")
        while not rospy.is_shutdown():
            self.complex_compute()
            rospy.sleep(1.0)

    def complex_compute(self):
        self.rrt1.publish("robot 1 is alive")


if __name__ == '__main__':
    rospy.init_node('robot_1')
    robot1 = Robot()
    robot1.robot_main()

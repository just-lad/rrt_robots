#!/usr/bin/env python

import roslib
import rospy
from jetbot import Robot
from std_msgs.msg import String
from rrt_robots.msg import move_command_struct


class RosCommander:
    def __init__(self):
        self.command_sub = rospy.Subscriber('/move_commands', move_command_struct, self.isr_commander)
        self.left = 0
        self.right = 0
        self.exec_time = 0
        self.new_command = False

    def isr_commander(self, command):
        self.left = command.left_motor
        self.right = command.right_motor
        self.exec_time = command.exec_time
        self.new_command = True


def main():
    ros = RosCommander()
    robot = Robot()

    rospy.init_node('Robot_1')

    while not rospy.is_shutdown:
        if ros.new_command:
            robot.set_motors(ros.left, ros.right)
            rospy.sleep(ros.exec_time)
            robot.set_motors(0, 0)
            ros.new_command = False
        else:
            pass

    rospy.spin()


if __name__ == '__main__':
    main()

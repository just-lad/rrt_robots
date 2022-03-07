#!/usr/bin/env python

import roslib
import rospy
from jetbot import Robot
from std_msgs.msg import String
from rrt_robots.msg import move_command_struct
import RPi.GPIO as GPIO
from math import radians, pi

wheel_base = 130
wheel_radius = 25
wheel_holes = 8
actual_ID = 6
# minimum angle delta to react to
angle_threshold = (float(wheel_radius) / float(wheel_base)) * radians(360.0/wheel_holes)
distance_threshold = round((2*pi*wheel_radius/wheel_holes)*0.4, 0)


class RobotMover:

    def __init__(self):
        # pins which sensors are connected to
        self.left_sensor_pin = 12
        self.right_sensor_pin = 13
        # vars to contain actual position
        self.robotID = 0
        self.angle_delta = 0
        self.dist_to_goal = 0
        # counters to allow step mode work properly
        self.counterL = 0
        self.counterR = 0
        # params to constrain movement
        self.robotID = 0
        self.max_speed = 0.3
        self.max_move_steps = 10
        self.max_spin_steps = 2

    def ISR_commander(self, command):
        # each ros tick renew actual position
        self.robotID = command.robot_ID
        self.angle_delta = command.angle_delta
        self.dist_to_goal = command.dist_to_goal

    def ISR_CountL(self, channel):
        # each left wheel tick increase counter
        self.counterL += 1

    def ISR_CountR(self, channel):
        # each right wheel tick increase counter
        self.counterR += 1

    def spin_l(self, steps, speed, robot):
        '''
        :param steps: steps to spin anti-clockwise
        :param speed: speed to go with
        :param robot: executing object
        :return: True if success
        '''
        self.counterL = 0
        self.counterR = 0

        while steps > self.counterL and steps > self.counterR:

            if steps > self.counterL:
                robot.left_motor.value = -speed
            else:
                robot.left_motor.value = 0

            if steps > self.counterR:
                robot.right_motor.value = speed
            else:
                robot.right_motor.value = 0
        return True

    def spin_r(self, steps, speed, robot):
        '''
        :param steps: steps to spin clockwise
        :param speed: speed to go with
        :param robot: executing object
        :return: True if success
        '''
        self.counterL = 0
        self.counterR = 0

        while steps > self.counterL and steps > self.counterR:

            if steps > self.counterL:
                robot.left_motor.value = speed
            else:
                robot.left_motor.value = 0

            if steps > self.counterR:
                robot.right_motor.value = -speed
            else:
                robot.right_motor.value = 0
        return True

    def go_forward(self, steps, speed, robot):
        '''
        :param steps: steps to go forward
        :param speed: speed to go with
        :param robot: executing object
        :return: True if success
        '''
        self.counterL = 0
        self.counterR = 0

        while steps > self.counterL and steps > self.counterR:

            if steps > self.counterL:
                robot.left_motor.value = speed
            else:
                robot.left_motor.value = 0

            if steps > self.counterR:
                robot.right_motor.value = speed
            else:
                robot.right_motor.value = 0
        return True

    def seek(self, robot, robot_id):
        '''
        :param robot: executing object
        :param robot_id: robot to execute command
        :return: True if success
        '''
        if robot_id != actual_ID:
            return False
        if self.angle_delta % angle_threshold > 0:              # if angle resolution is enough
            if self.angle_delta < pi:                           # if should spin clockwise
                step = constrain((self.angle_delta // angle_threshold), 1, self.max_spin_steps)
                self.spin_r(step, self.max_speed, robot)
                return True
            if self.angle_delta >= pi:                          # if should spin anti-clockwise
                step = constrain(((self.angle_delta - pi) // angle_threshold), 1, self.max_spin_steps)
                self.spin_l(step, self.max_speed, robot)
                return True
        elif self.dist_to_goal % distance_threshold > 0:        # if linear resolution is enough
            step = constrain((self.dist_to_goal // distance_threshold), 1, self.max_move_steps)
            self.go_forward(step, self.max_speed, robot)
            return True
        else:
            return False


def constrain(val, min_val, max_val):
    '''
    :param val: value
    :param min_val: minimum allowed value
    :param max_val: maximum allowed value
    :return: constrained value
    '''
    return min(max_val, max(min_val, val))


def main():
    robot = Robot()
    mover = RobotMover()

    GPIO.setmode(GPIO.BOARD)                # BOARD pin-numbering scheme
    GPIO.setup(mover.left_sensor_pin, GPIO.IN)    # left_sensor_pin set as input
    GPIO.setup(mover.right_sensor_pin, GPIO.IN)   # right_sensor_pin set as input

    GPIO.add_event_detect(mover.left_sensor_pin, GPIO.RISING, callback=mover.ISR_CountL, bouncetime=10)
    GPIO.add_event_detect(mover.right_sensor_pin, GPIO.RISING, callback=mover.ISR_CountR, bouncetime=10)

    rospy.init_node('Robot_1')
    rospy.Subscriber('/move_commands', move_command_struct, mover.ISR_commander)

    while not rospy.shutdown():
        while mover.seek(robot, mover.robotID):
            pass


if __name__ == '__main__':
    main()

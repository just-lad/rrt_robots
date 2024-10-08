#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from math import pi
from rrt_robots.msg import move_command_struct

import rrt_connect as rrt
import utilities as ut

distance_threshold = 40
angle_threshold = 0.3
delay = 2.0
start_id = 2
goal_id = 5


class RobotPlanner:

    def __init__(self):
        self.draw_flag = True

        self.image_path_pub = rospy.Publisher("/marks_obs_path", Image, queue_size=1)
        self.command_pub = rospy.Publisher("/move_commands", move_command_struct, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw/", Image, self.callback)
        self.command = move_command_struct()

        self.aruco_corners = None
        self.first_frame = None

        self.path = None
        self.tree1 = None
        self.tree2 = None
        self.custom_obstacles_list = []

        self.max_speed = 0.4
        self.turn_time = 0.1
        self.forward_time = 0.2

    def initialize_scene(self):
        """
        :return: void
        """
        blurred_input = cv2.GaussianBlur(self.first_frame, (5, 5), 0)
        gray = cv2.cvtColor(blurred_input, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 120, 250, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            if ((70 < w < 100) and (130 < h < 500)) or ((130 < w < 500) and (70 < h < 100)):
                rect = x, y, w, h
                self.custom_obstacles_list.append(rect)
        start = ut.get_pose(self.aruco_corners, start_id)
        goal = ut.get_pose(self.aruco_corners, goal_id)
        robot_start = (int(start[4][0]), int(start[4][1]))
        robot_goal = (int(goal[4][0]), int(goal[4][1]))

        custom_env = rrt.Env(self.first_frame.shape[1],
                             self.first_frame.shape[0],
                             self.custom_obstacles_list)
        rrt_conn = rrt.RrtConnect(robot_start, robot_goal, 40, 0.2, 5000, custom_env)
        self.path = rrt_conn.planning()
        self.tree1 = rrt_conn.V1
        self.tree2 = rrt_conn.V2
        return True

    def callback(self, data):
        """
        :param data: data received from subscriber
        :return: nothing, just processes data and write it to self
        """

        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        if self.first_frame is None and cv_image is not None:
            self.first_frame = cv_image

        if self.draw_flag:
            markers_img, self.aruco_corners = ut.draw_aruco(cv_image)

            if self.path is not None:
                ut.draw_whole_tree(self.tree1, markers_img, (255, 0, 0))
                ut.draw_whole_tree(self.tree2, markers_img, (255, 0, 0))
                ut.draw_path(self.path, markers_img, (0, 255, 0))
                ut.draw_obstacles(self.custom_obstacles_list, markers_img, (0, 255, 0))

            if self.path is not None:
                self.image_path_pub.publish(self.bridge.cv2_to_imgmsg(markers_img, "bgr8"))

        else:
            _, self.aruco_corners = ut.draw_aruco(cv_image)

    def move_to_point(self, point, robot_id):
        """
        :param robot_id: robot ID to send command to
        :param point: point to move to
        :return: True if reached
        """
        if robot_id is not None:
            self.command.robot_ID = robot_id
            robot_cords = ut.get_pose(self.aruco_corners, robot_id)
            if robot_cords is None:
                return False

            angle_delta = ut.angle_between_three(robot_cords[5], robot_cords[4], point)
            dist_to_goal = ut.distance_between_points(robot_cords[4], point)

            if dist_to_goal < distance_threshold:
                self.stop()
                return True

            if angle_delta > angle_threshold and abs(angle_delta - 2 * pi) > angle_threshold:
                if pi > angle_delta:
                    self.turn_left()
                    return False
                else:
                    self.turn_right()
                    return False
            else:
                self.go_forward()
                return False

    def turn_left(self):
        self.write_command(-self.max_speed, self.max_speed, self.turn_time)

    def turn_right(self):
        self.write_command(self.max_speed, -self.max_speed, self.turn_time)

    def go_forward(self):
        self.write_command(self.max_speed, 0.9*self.max_speed, self.forward_time)

    def stop(self):
        self.write_command(0, 0, self.forward_time)

    def write_command(self, left, right, time):
        """
        :param left: left motor signal, float, 0 to 1
        :param right: right motor signal, float, 0 to 1
        :param time: execution of the command time
        :return:
        """
        self.command.left_motor = left
        self.command.right_motor = right
        self.command.exec_time = time

    def stop_robots(self):
        """
        :return: True if success
        """
        self.write_command(0, 0, 1)
        self.command.robot_ID = start_id
        self.command_pub.publish(self.command)
        self.command.robot_ID = goal_id
        self.command_pub.publish(self.command)
        return True


def main():
    print("Initializing master_node")
    rospy.init_node('Master_node', anonymous=True)
    print("Master launched")
    planner = RobotPlanner()

    while True:
        if planner.first_frame is not None and planner.aruco_corners is not None:
            planner.initialize_scene()
            robot_1_path, robot_2_path = ut.split_path(planner.path)
            r1_index = 0
            r2_index = 0
            break

    rate = rospy.Rate(1/delay)
    while not rospy.is_shutdown():

        if planner.move_to_point(robot_1_path[r1_index], start_id):
            if r1_index < len(robot_1_path) - 1:
                r1_index += 1
        else:
            planner.command_pub.publish(planner.command)

        if planner.move_to_point(robot_2_path[r2_index], goal_id):
            if r2_index < len(robot_2_path) - 1:
                r2_index += 1
        else:
            planner.command_pub.publish(planner.command)

        if r1_index == len(robot_1_path) - 1 and r2_index == len(robot_2_path) - 1:
            planner.stop_robots()
            break

        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    main()

#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
from aruco_detection.msg import move_command_struct
from math import atan2, sqrt, degrees, radians
import rrt_connect as rrt

distance_threshold = 20
target_robot = 6
custom_goal = (600, 600)


class ArucoDetector:

    def __init__(self):
        self.image_pub = rospy.Publisher("/detected_markers", Image, queue_size=1)
        self.command_pub = rospy.Publisher("/move_commands", move_command_struct, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw/", Image, self.callback)
        self.command = move_command_struct()
        self.aruco_corners = None
        self.goal_i = 0
        self.path = None
        self.first_frame = None
        self.robot_center_init = None

    def callback(self, data):
        '''
        :param data: data received from subscriber
        :return: nothing, just processes data and write it to self
        '''
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.first_frame is None and cv_image is not None:
            self.first_frame = cv_image

        markers_img, self.aruco_corners = self.draw_aruco(cv_image)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(markers_img, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def findArucoCoords(self, bbox):
        '''
        :param bbox: list with aruco corners
        :return: list of coordinates
        '''
        topl = bbox[0][0][0], bbox[0][0][1]
        botl = bbox[0][3][0], bbox[0][3][1]
        topr = bbox[0][1][0], bbox[0][1][1]
        botr = bbox[0][2][0], bbox[0][2][1]

        center = ((topl[0] + botr[0]) // 2,
                  (topl[1] + botr[1]) // 2)

        mid_top = ((topl[0] + topr[0]) // 2, (topl[1] + topr[1]) // 2)
        mid_bot = ((botl[0] + botr[0]) // 2, (botl[1] + botr[1]) // 2)
        mid_left = ((topl[0] + botl[0]) // 2, (topl[1] + botl[1]) // 2)
        mid_right = ((topr[0] + botr[0]) // 2, (topr[1] + botr[1]) // 2)

        coords = [topl,       # 0 (X Y)
                  topr,       # 1 (X Y)
                  botl,       # 2 (X Y)
                  botr,       # 3 (X Y)
                  center,     # 4 (X Y)
                  mid_top,    # 5 (X Y)
                  mid_bot,    # 6 (X Y)
                  mid_left,   # 7 (X Y)
                  mid_right]  # 8 (X Y)

        return coords

    def draw_aruco(self, img):
        '''
        :param img: image to detect aruco in
        :return: img with drawn aruco and list of two [corners, ids]
        '''
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        output = aruco.drawDetectedMarkers(img, corners, ids)  # detect the Aruco markers and display its aruco id.
        return output, [corners, ids]

    def move_to_point(self, point):
        '''
        :param point: point to move to
        :return: True if reached
        '''
        robot_id = None
        for ids in self.aruco_corners[1][0]:
            if self.aruco_corners[1][0][ids] == target_robot:
                robot_id = ids

        if robot_id is not None:
            self.command.robot_ID = robot_id
            robot_coords = self.findArucoCoords(self.aruco_corners[0][0])

            if self.robot_center_init is None:
                self.robot_center_init = robot_coords[4]

            self.command.angle_delta = angle_between_three(robot_coords[5], robot_coords[4], point)
            self.command.dist_to_goal = distance_between_points(robot_coords[4], point)

            if self.command.dist_to_goal < distance_threshold:
                self.command.dist_to_goal = 0
                self.command_pub.publish(self.command)
                return True
            else:
                self.command_pub.publish(self.command)
                return False
        else:
            return False

    def follow_path(self, path):
        '''
        :param path: list of tuples (x,y), represent path's waypoints
        :return: True if goal is reached
        '''
        for current_point in path:
            while not self.move_to_point(path[current_point]):
                pass
        return True


def angle_between_three(p1, p2, p3):
    '''
    :param p1: some point (x, y)
    :param p2: angle center point (x, y)
    :param p3: some point (x, y)
    :return: angle in radians, positive, anti-clockwise
    '''
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    deg1 = (360 + degrees(atan2(x1 - x2, y1 - y2))) % 360
    deg2 = (360 + degrees(atan2(x3 - x2, y3 - y2))) % 360
    return radians(deg2 - deg1) if deg1 <= deg2 else radians(360 - (deg1 - deg2))


def distance_between_points(point1, point2):
    '''
    :param point1: some point (x, y)
    :param point2: some point (x, y)
    :return: distance, integer
    '''
    return round(sqrt(
            ((point1[0] - point2[0]) ** 2) + ((point2[1] - point2[1]) ** 2)), 0)


def constrain(val, min_val, max_val):
    '''
    :param val: your value to constarin
    :param min_val: desired minimum limit
    :param max_val: desired maximum limit
    :return: constrained value
    '''
    return min(max_val, max(min_val, val))


def if_exist(object):
    '''
    :param object: object to check existance
    :return: True if exists
    '''
    try:
        object
        return True
    except:
        return False


def main():
    custom_rects_list = []
    path = None
    goal_reached = False

    print("Initializing detect_markers")
    rospy.init_node('detect_markers', anonymous=True)
    print("Bring the aruco-ID in front of camera")
    detector = ArucoDetector()

    if not if_exist(rrt_conn):
        custom_x_range = detector.first_frame.shape[1]
        custom_y_range = detector.first_frame.shape[0]
        blurred_input = cv2.GaussianBlur(detector.first_frame, (5, 5), 0)
        gray = cv2.cvtColor(blurred_input, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 150, 250, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            rect = cv2.boundingRect(c)
            if rect[2] < 60 or rect[3] < 80:
                continue
            custom_rects_list.append(rect)
        custom_start = detector.robot_center_init
        custom_env = rrt.Env(custom_x_range, custom_y_range, custom_rects_list)
        rrt_conn = rrt.RrtConnect(custom_start, custom_goal, 60, 0.1, 5000, custom_env)
        path = rrt_conn.planning()  # list of tuples [(x1, y1), (x2, y2)... (x_goal, y_goal)]

    if detector.follow_path(path):
        goal_reached = True

    # if detector.aruco_corners is not None:
    #     if not if_exist(rrt_conn):
    #         start_coords = detector.findArucoCoords(detector.aruco_corners)
    #         rrt_conn = RrtConnect(start_coords, goal_coords, 50, 0.05, 5000)
    #         detector.path = rrt_conn.planning()

    while not goal_reached:
        rospy.spin()


if __name__ == '__main__':
    main()


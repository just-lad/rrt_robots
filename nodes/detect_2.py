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
from itertools import tee, izip

distance_threshold = 20
start_id = 2
goal_id = 5


class ArucoDetector:

    def __init__(self):
        self.image_pub = rospy.Publisher("/detected_markers", Image, queue_size=1)
        self.image_path_pub = rospy.Publisher("/marks_obs_path", Image, queue_size=1)
        self.command_pub = rospy.Publisher("/move_commands", move_command_struct, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw/", Image, self.callback)
        self.command = move_command_struct()
        self.aruco_corners = None
        self.path = None
        self.first_frame = None
        self.custom_start = None
        self.custom_goal = None
        self.path_to_draw = None
        self.custom_rects_list = None

    def init_scene(self):
        blurred_input = cv2.GaussianBlur(self.first_frame, (5, 5), 0)
        gray = cv2.cvtColor(blurred_input, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 150, 250, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            rect = cv2.boundingRect(c)
            if rect[2] < 60 or rect[3] < 80:
                self.custom_rects_list.append(rect)
        start = self.get_pose(self.aruco_corners, start_id)
        goal = self.get_pose(self.aruco_corners, goal_id)
        self.custom_start = start[4]
        self.custom_goal = goal[4]

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

        if self.path_to_draw is not None:
            image_with_path = draw_path(markers_img)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(markers_img, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def draw_path(self, img):
        paired_path = pairwise(self.path_to_draw)
        for i in paired_path:
            cv2.line(img, paired_path[i][0][1], (0, 255, 0), thickness=3)
        return img

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

    def move_to_point(self, point, id):
        '''
        :param point: point to move to
        :return: True if reached
        '''

        if id is not None:
            self.command.robot_ID = id
            robot_coords = self.get_pose(self.aruco_corners, id)
            if robot_coords is None:
                return False

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

    def get_pose(self, aruco_list, id):
        '''
        :param id: aruco id to get pose of
        :param aruco_list: list with aruco marker corners and ids [corners, ids]
        :return: aruco center
        '''
        for ids in range(len(aruco_list[1])):
            if aruco_list[1][ids] == id:
                aruco_coords = self.findArucoCoords(aruco_list[0][ids])
                return aruco_coords
        return None


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


def pairwise(iterable):
    '''
    :param iterable: list to iterate by 2
    :return: list with tuples by 2
    '''
    a, b = tee(iterable)
    next(b, None)
    return list(izip(a, b))


def split_path(path):
    '''
    :param path: path to split in two
    :return: two lists, represents to paths to the central point
    '''
    half = len(path)//2
    return path[:half], list(reversed(path[half:]))


def main():
    print("Initializing detect_markers")
    rospy.init_node('detect_markers', anonymous=True)
    print("Bring the aruco-ID in front of camera")
    detector = ArucoDetector()

    while detector.first_frame is None:
        if detector.first_frame is not None:
            detector.init_scene()
            custom_env = rrt.Env(detector.first_frame.shape[1], detector.first_frame.shape[0], detector.custom_rects_list)
            rrt_conn = rrt.RrtConnect(detector.custom_start, detector.custom_goal, 60, 0.1, 5000, custom_env)
            path = rrt_conn.planning()  # list of tuples [(x1, y1), (x2, y2)... (x_goal, y_goal)]
            robot_1_path, robot_2_path = split_path(path)
            r1_index = 0
            r2_index = 0

    while True:
        if r1_index > len(robot_1_path):
            break
        if detector.move_to_point(robot_1_path[r1_index], start_id):
            r1_index += 1
        if r2_index > len(robot_2_path):
            break
        if detector.move_to_point(robot_2_path[r2_index], goal_id):
            r2_index += 1

    rospy.spin()


if __name__ == '__main__':
    main()


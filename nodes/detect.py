#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
from rrt_robots.msg import move_command_struct
from math import atan2, sqrt, degrees, radians, pi

goal_coords = [(300, 300),
               (600, 600)]


class ArucoDetector:

    def __init__(self):
        self.image_pub = rospy.Publisher("/detected_markers", Image, queue_size=1)
        self.command_pub = rospy.Publisher("/move_commands", move_command_struct, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw/", Image, self.callback)
        self.command = move_command_struct()
        self.aruco_corners = None
        self.point_reached = False
        self.goal_i = 0
        self.path = None
        self.max_speed = 0.36
        self.turn_time = 0.1
        self.forw_time = 0.3

    def callback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        markers_img, self.aruco_corners = self.draw_aruco(cv_image)

        if self.point_reached:
            if self.goal_i == 0:
                self.goal_i = 1
            else:
                self.goal_i = 0
            self.point_reached = False
            pass
        if self.move_to_point(goal_coords[self.goal_i]):
            self.point_reached = True

        cv2.circle(markers_img, goal_coords[self.goal_i], 30, (0, 0, 255), thickness=3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(markers_img, "bgr8"))
        except CvBridgeError as e:
            print(e)


    def findArucoCoords(self, bbox):
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
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        output = aruco.drawDetectedMarkers(img, corners, ids)  # detect the Aruco markers and display its aruco id.
        return output, [corners, ids]

    def write_command(self, left, right, time):
        self.command.left_motor = left
        self.command.right_motor = right
        self.command.exec_time = time

    def move_to_point(self, point):
        robot_coords = self.findArucoCoords(self.aruco_corners[0][0])

        angle_delta = angle_between_three(robot_coords[5], robot_coords[4], point)
        dist_to_goal = distance_between_points(robot_coords[4], point)
        self.command.angle = angle_delta

        if dist_to_goal < 30:
            self.write_command(0, 0, self.forw_time)
            return True

        if angle_delta > 0.3 and abs(angle_delta - 2*pi) > 0.3:
            if angle_delta < pi:
                self.write_command(-self.max_speed, self.max_speed, self.turn_time)
                return False
            else:
                self.write_command(self.max_speed, -self.max_speed, self.turn_time)
                return False
        else:
            self.write_command(self.max_speed, self.max_speed, self.forw_time)
            return False


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
    return min(max_val, max(min_val, val))


def main():
    print("Initializing detect_markers")
    rospy.init_node('detect_markers', anonymous=True)
    print("Bring the aruco-ID in front of camera")
    detector = ArucoDetector()

    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        detector.command_pub.publish(detector.command)
        rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    main()


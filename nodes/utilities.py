import cv2.aruco as aruco
import cv2
from math import atan2, sqrt, degrees, radians
from itertools import tee, izip


def draw_obstacles(obstacles_list, image, (b, g, r)):
    """
    :param obstacles_list: obstacles to draw in format (x, y, width, heights)
    :param image: image to draw on
    :return: image with obstacles
    """
    for i in range(len(obstacles_list)):
        x, y, w, h = obstacles_list[i]
        cv2.rectangle(image,
                      (x, y),
                      (x + w, y + h),
                      (b, g, r),
                      thickness=2)
    return image


def draw_path(path, image, (b, g, r)):
    """
    :param path: path to draw
    :param image: img to draw on
    :return: image with path and docking point
    """
    paired_path = pairwise(path)
    for i in range(len(paired_path)):
        cv2.line(image,
                 paired_path[i][0],
                 paired_path[i][1],
                 (0, 255, 0),
                 thickness=2)
    path1, path2 = split_path(path)
    docking_point = ((path1[len(path1) - 1][0] + path2[len(path2) - 1][0]) // 2,
                     (path1[len(path1) - 1][1] + path2[len(path2) - 1][1]) // 2)
    cv2.circle(image,
               docking_point,
               20,
               (b, g, r),
               thickness=3)
    return image


def draw_whole_tree(node_list, image, (b, g, r)):
    """
    :param node_list: list of nodes from rrt
    :param image: image to draw on
    :return: image with tree
    """
    for k in range(len(node_list)):
        if k < len(node_list):
            if node_list[k].parent:
                cv2.line(image,
                         (int(node_list[k].x), int(node_list[k].y)),
                         (int(node_list[k].parent.x), int(node_list[k].parent.y)),
                         (b, g, r),
                         thickness=2)
    return image


def get_pose(aruco_list, aruco_id):
    """
    :param aruco_id: aruco id to get pose of
    :param aruco_list: list with aruco marker corners and ids [corners, ids]
    :return: aruco center
    """
    for ids in range(len(aruco_list[1])):
        if aruco_list[1][ids] == aruco_id:
            aruco_cords = find_aruco_cords(aruco_list[0][ids])
            return aruco_cords
    return None


def find_aruco_cords(bbox):
    """
    :param bbox: list with aruco corners
    :return: list of coordinates
    """
    top_left = bbox[0][0][0], bbox[0][0][1]
    bot_left = bbox[0][3][0], bbox[0][3][1]
    top_right = bbox[0][1][0], bbox[0][1][1]
    bot_right = bbox[0][2][0], bbox[0][2][1]

    center = ((top_left[0] + bot_right[0]) // 2,
              (top_left[1] + bot_right[1]) // 2)

    mid_top = ((top_left[0] + top_right[0]) // 2,
               (top_left[1] + top_right[1]) // 2)
    mid_bot = ((bot_left[0] + bot_right[0]) // 2,
               (bot_left[1] + bot_right[1]) // 2)
    mid_left = ((top_left[0] + bot_left[0]) // 2,
                (top_left[1] + bot_left[1]) // 2)
    mid_right = ((top_right[0] + bot_right[0]) // 2,
                 (top_right[1] + bot_right[1]) // 2)

    cords = [top_left,      # 0 (X Y)
             top_right,     # 1 (X Y)
             bot_left,      # 2 (X Y)
             bot_right,     # 3 (X Y)
             center,        # 4 (X Y)
             mid_top,       # 5 (X Y)
             mid_bot,       # 6 (X Y)
             mid_left,      # 7 (X Y)
             mid_right]     # 8 (X Y)

    return cords


def draw_aruco(img):
    """
    :param img: image to detect aruco in
    :return: img with drawn aruco and list of two [corners, ids]
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray,
                                          aruco_dict,
                                          parameters=parameters)
    output = aruco.drawDetectedMarkers(img, corners, ids)  # detect the Aruco markers and display its aruco id.
    return output, [corners, ids]


def angle_between_three(p1, p2, p3):
    """
    :param p1: some point (x, y)
    :param p2: angle center point (x, y)
    :param p3: some point (x, y)
    :return: angle in radians, positive, anti-clockwise
    """
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    deg1 = (360 + degrees(atan2(x1 - x2, y1 - y2))) % 360
    deg2 = (360 + degrees(atan2(x3 - x2, y3 - y2))) % 360
    return radians(deg2 - deg1) if deg1 <= deg2 else radians(360 - (deg1 - deg2))


def distance_between_points(point1, point2):
    """
    :param point1: some point (x, y)
    :param point2: some point (x, y)
    :return: distance, integer
    """
    return round(sqrt(
        ((point1[0] - point2[0]) ** 2) + ((point2[1] - point2[1]) ** 2)), 0)


def pairwise(iterable):
    """
    :param iterable: list to iterate by 2
    :return: list with tuples by 2
    """
    a, b = tee(iterable)
    next(b, None)
    return list(izip(a, b))


def split_path(path):
    """
    :param path: path to split in two
    :return: two lists, represents two paths to the central point
    """
    half = len(path) // 2
    return path[:half], list(reversed(path[half:]))

import cv2.aruco as aruco
import cv2
from math import atan2, sqrt, degrees, radians
from itertools import tee, izip


def get_pose(aruco_list, id):
    """
    :param id: aruco id to get pose of
    :param aruco_list: list with aruco marker corners and ids [corners, ids]
    :return: aruco center
    """
    for ids in range(len(aruco_list[1])):
        if aruco_list[1][ids] == id:
            aruco_coords = find_aruco_coords(aruco_list[0][ids])
            return aruco_coords
    return None


def find_aruco_coords(bbox):
    """
    :param bbox: list with aruco corners
    :return: list of coordinates
    """
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


def draw_aruco(img):
    """
    :param img: image to detect aruco in
    :return: img with drawn aruco and list of two [corners, ids]
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
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
    half = len(path)//2
    return path[:half], list(reversed(path[half:]))

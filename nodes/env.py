"""
Environment for rrt_2D
@author: huiming zhou
"""


class Env:
    def __init__(self):
        self.x_range = (0, 720)
        self.y_range = (0, 1000)
        self.obs_boundary = self.obs_boundary()
        self.obs_circle = self.obs_circle()
        self.obs_rectangle = self.obs_rectangle()

    @staticmethod
    def obs_boundary():
        obs_boundary = [
            [0, 0, 720, 1000]
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle():
        obs_rectangle = [
            [0, 0, 0, 0]
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            [0, 0, 0]
        ]

        return obs_cir

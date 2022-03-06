import cv2
import sys
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os

image_to_process = "5.jpg"

class Env:
    def __init__(self, x_rng, y_rng, rect_list):
        self.x_range = (0, x_rng)
        self.y_range = (0, y_rng)
        self.obs_boundary = self.obs_boundary()
        self.obs_circle = self.obs_circle()
        self.obs_rectangle = rect_list

    @staticmethod
    def obs_boundary():
        obs_boundary = None
        return obs_boundary

    @staticmethod
    def obs_rectangle():
        obs_rectangle = None
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = None
        return obs_cir


class Plotting:
    def __init__(self, x_start, x_goal, env):
        self.xI, self.xG = x_start, x_goal
        self.env = env
        self.obs_bound = self.env.obs_boundary
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle

    def animation(self, nodelist, path, name, animation=False):
        self.plot_grid(name)
        self.plot_visited(nodelist, animation)
        self.plot_path(path)

    def animation_connect(self, V1, V2, path, name):
        self.plot_grid(name)
        self.plot_visited_connect(V1, V2)
        self.plot_path(path)

    def plot_grid(self, name):
        fig, ax = plt.subplots()
        
        if self.obs_bound is not None:
            for (ox, oy, w, h) in self.obs_bound:
                ax.add_patch(
                   patches.Rectangle(
                        (ox, oy), w, h,
                        edgecolor='black',
                        facecolor='black',
                        fill=True
                    )
                )

        if self.obs_rectangle is not None:
            for (ox, oy, w, h) in self.obs_rectangle:
                ax.add_patch(
                    patches.Rectangle(
                        (ox, oy), w, h,
                        edgecolor='black',
                        facecolor='gray',
                        fill=True
                    )
                )

        if self.obs_circle is not None:
            for (ox, oy, r) in self.obs_circle:
                ax.add_patch(
                    patches.Circle(
                        (ox, oy), r,
                        edgecolor='black',
                        facecolor='gray',
                        fill=True
                    )
                )

        plt.plot(self.xI[0], self.xI[1], "bs", linewidth=3)
        plt.plot(self.xG[0], self.xG[1], "gs", linewidth=3)

        plt.title(name)
        #plt.axis("equal")
        plt.xlim(self.env.x_range)
        plt.ylim(self.env.y_range)
        

    @staticmethod
    def plot_visited(nodelist, animation):
        if animation:
            count = 0
            for node in nodelist:
                count += 1
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])
                    if count % 10 == 0:
                        plt.pause(0.1)
                        pass
        else:
            for node in nodelist:
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")

    @staticmethod
    def plot_visited_connect(V1, V2):
        len1, len2 = len(V1), len(V2)

        for k in range(max(len1, len2)):
            if k < len1:
                if V1[k].parent:
                    plt.plot([V1[k].x, V1[k].parent.x], [V1[k].y, V1[k].parent.y], "-g")
            if k < len2:
                if V2[k].parent:
                    plt.plot([V2[k].x, V2[k].parent.x], [V2[k].y, V2[k].parent.y], "-g")

            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            if k % 2 == 0:
                #plt.pause(0.1)
                pass

        #plt.pause(0.01)
        pass

    @staticmethod
    def plot_path(path):
        if len(path) != 0:
            plt.plot([x[0] for x in path], [x[1] for x in path], '-r', linewidth=2)
            #plt.pause(0.01)
        plt.show()
        

class Utils:
    def __init__(self, env):
        self.env = env

        self.delta = 30
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

    def update_obs(self, obs_cir, obs_bound, obs_rec):
        self.obs_circle = obs_cir
        self.obs_boundary = obs_bound
        self.obs_rectangle = obs_rec

    def get_obs_vertex(self):
        delta = self.delta
        obs_list = []
        if self.obs_rectangle is not None:
            for (ox, oy, w, h) in self.obs_rectangle:
                vertex_list = [[ox - delta, oy - delta],
                               [ox + w + delta, oy - delta],
                               [ox + w + delta, oy + h + delta],
                               [ox - delta, oy + h + delta]]
                obs_list.append(vertex_list)

        return obs_list

    def is_intersect_rec(self, start, end, o, d, a, b):
        v1 = [o[0] - a[0], o[1] - a[1]]
        v2 = [b[0] - a[0], b[1] - a[1]]
        v3 = [-d[1], d[0]]

        div = np.dot(v2, v3)

        if div == 0:
            return False

        t1 = np.linalg.norm(np.cross(v2, v1)) / div
        t2 = np.dot(v1, v3) / div

        if t1 >= 0 and 0 <= t2 <= 1:
            shot = Node((o[0] + t1 * d[0], o[1] + t1 * d[1]))
            dist_obs = self.get_dist(start, shot)
            dist_seg = self.get_dist(start, end)
            if dist_obs <= dist_seg:
                return True

        return False

    def is_intersect_circle(self, o, d, a, r):
        d2 = np.dot(d, d)
        delta = self.delta

        if d2 == 0:
            return False

        t = np.dot([a[0] - o[0], a[1] - o[1]], d) / d2

        if 0 <= t <= 1:
            shot = Node((o[0] + t * d[0], o[1] + t * d[1]))
            if self.get_dist(shot, Node(a)) <= r + delta:
                return True

        return False

    def is_collision(self, start, end):
        if self.is_inside_obs(start) or self.is_inside_obs(end):
            return True

        o, d = self.get_ray(start, end)
        obs_vertex = self.get_obs_vertex()

        for (v1, v2, v3, v4) in obs_vertex:
            if self.is_intersect_rec(start, end, o, d, v1, v2):
                return True
            if self.is_intersect_rec(start, end, o, d, v2, v3):
                return True
            if self.is_intersect_rec(start, end, o, d, v3, v4):
                return True
            if self.is_intersect_rec(start, end, o, d, v4, v1):
                return True
        if self.obs_circle is not None:
            for (x, y, r) in self.obs_circle:
                if self.is_intersect_circle(o, d, [x, y], r):
                    return True

        return False

    def is_inside_obs(self, node):
        delta = self.delta
        if self.obs_circle is not None:
            for (x, y, r) in self.obs_circle:
                if math.hypot(node.x - x, node.y - y) <= r + delta:
                    return True
        if self.obs_rectangle is not None:
            for (x, y, w, h) in self.obs_rectangle:
                if 0 <= node.x - (x - delta) <= w + 2 * delta \
                        and 0 <= node.y - (y - delta) <= h + 2 * delta:
                    return True
        if self.obs_boundary is not None:
            for (x, y, w, h) in self.obs_boundary:
                if 0 <= node.x - (x - delta) <= w + 2 * delta \
                        and 0 <= node.y - (y - delta) <= h + 2 * delta:
                    return True

        return False

    @staticmethod
    def get_ray(start, end):
        orig = [start.x, start.y]
        direc = [end.x - start.x, end.y - start.y]
        return orig, direc

    @staticmethod
    def get_dist(start, end):
        return math.hypot(end.x - start.x, end.y - start.y)
    
class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class RrtConnect:
    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, iter_max, env):

        self.s_start = Node(s_start)                # starting point node
        self.s_goal = Node(s_goal)                  # goal point node
        self.step_len = step_len                    # length of tree branches
        self.goal_sample_rate = goal_sample_rate    # probability of sampling the goal as a new node
        self.iter_max = iter_max                    # max number of iterations without solution
        self.V1 = [self.s_start]
        self.V2 = [self.s_goal]

        self.env = env
        self.plotting = Plotting(s_start, s_goal, env)
        self.utils = Utils(env)

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

    def planning(self):
        for i in range(self.iter_max):
            node_rand = self.generate_random_node(self.s_goal, self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.V1, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and not self.utils.is_collision(node_near, node_new):
                self.V1.append(node_new)
                node_near_prim = self.nearest_neighbor(self.V2, node_new)
                node_new_prim = self.new_state(node_near_prim, node_new)

                if node_new_prim and not self.utils.is_collision(node_new_prim, node_near_prim):
                    self.V2.append(node_new_prim)

                    while True:
                        node_new_prim2 = self.new_state(node_new_prim, node_new)
                        if node_new_prim2 and not self.utils.is_collision(node_new_prim2, node_new_prim):
                            self.V2.append(node_new_prim2)
                            node_new_prim = self.change_node(node_new_prim, node_new_prim2)
                        else:
                            break

                        if self.is_node_same(node_new_prim, node_new):
                            break

                if self.is_node_same(node_new_prim, node_new):
                    return self.extract_path(node_new, node_new_prim)

            if len(self.V2) < len(self.V1):
                list_mid = self.V2
                self.V2 = self.V1
                self.V1 = list_mid

        return None

    @staticmethod
    def change_node(node_new_prim, node_new_prim2):
        node_new = Node((node_new_prim2.x, node_new_prim2.y))
        node_new.parent = node_new_prim

        return node_new

    @staticmethod
    def is_node_same(node_new_prim, node_new):
        if node_new_prim.x == node_new.x and \
                node_new_prim.y == node_new.y:
            return True

        return False

    def generate_random_node(self, sample_goal, goal_sample_rate):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return sample_goal

    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))
        node_new.parent = node_start

        return node_new

    @staticmethod
    def extract_path(node_new, node_new_prim):
        path1 = [(node_new.x, node_new.y)]
        node_now = node_new

        while node_now.parent is not None:
            node_now = node_now.parent
            path1.append((node_now.x, node_now.y))

        path2 = [(node_new_prim.x, node_new_prim.y)]
        node_now = node_new_prim

        while node_now.parent is not None:
            node_now = node_now.parent
            path2.append((node_now.x, node_now.y))

        return list(list(reversed(path1)) + path2)

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)


def main():
    custom_rects_list = []
    custom_x_range = None
    custom_y_range = None
    custom_start = None
    custom_goal = None
    
    img = cv2.imread(cv2.samples.findFile(image_to_process))
    if img is None:
        sys.exit("Could not read the image.")
    
    dsize = (int(0.6*img.shape[1]), int(0.6*img.shape[0]))

 
    input_sized = cv2.resize(img, dsize)
    cropped_sized_input = input_sized[0:553, 60:920]
    custom_x_range = cropped_sized_input.shape[1]
    custom_y_range = cropped_sized_input.shape[0]
    blurred_input = cv2.GaussianBlur(cropped_sized_input, (5, 5), 0)
    gray = cv2.cvtColor(blurred_input, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 150, 250, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        rect = cv2.boundingRect(c)
        if rect[2] < 60 or rect[3] < 80 : continue
        custom_rects_list.append(rect)
        x,y,w,h = rect
        cv2.rectangle(cropped_sized_input,(x,y),(x+w,y+h),(0,255,0),2)

    
    #cv2.imshow("Display detected obstacles", cropped_sized_input)
    plt.imshow(cropped_sized_input)
    plt.gca().invert_yaxis()
    custom_start = (683, 190)     # Starting node
    custom_goal = (160, 165)  # Goal node
    custom_env = Env(custom_x_range, custom_y_range, custom_rects_list)
    rrt_conn = RrtConnect(custom_start, custom_goal, 60, 0.5, 5000, custom_env)
    path = rrt_conn.planning()
    rrt_conn.plotting.animation_connect(rrt_conn.V1, rrt_conn.V2, path, "RRT_CONNECT")
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()



if __name__ == '__main__':
    main()
    


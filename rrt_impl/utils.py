import env
import math
import numpy as np
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = None


class Utils:
    def __init__(self):
        x_bounds = (0, 50)
        y_bounds = (0, 30)
        self.env = env.Env(x_bounds, y_bounds)

        self.delta = 0.5
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
            shot = Node(o[0] + t1 * d[0], o[1] + t1 * d[1])
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
            shot = Node(o[0] + t * d[0], o[1] + t * d[1])
            if self.get_dist(shot, Node(a[0], a[1])) <= r + delta:
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

        for (x, y, r) in self.obs_circle:
            if self.is_intersect_circle(o, d, [x, y], r):
                return True

        return False

    def is_inside_obs(self, node):
        delta = self.delta

        for (x, y, r) in self.obs_circle:
            if math.hypot(node.x - x, node.y - y) <= r + delta:
                return True

        for (x, y, w, h) in self.obs_rectangle:
            if 0 <= node.x - (x - delta) <= w + 2 * delta \
                    and 0 <= node.y - (y - delta) <= h + 2 * delta:
                return True

        for (x, y, w, h) in self.obs_boundary:
            if 0 <= node.x - (x - delta) <= w + 2 * delta \
                    and 0 <= node.y - (y - delta) <= h + 2 * delta:
                return True

        return False
    def is_inside_obs_alternative(self, coords):
        delta = self.delta

        for (x, y, r) in self.obs_circle:
            if math.hypot(coords[0] - x, coords[1] - y) <= r + delta:
                return True

        for (x, y, w, h) in self.obs_rectangle:
            if 0 <= coords[0] - (x - delta) <= w + 2 * delta \
                    and 0 <= coords[1] - (y - delta) <= h + 2 * delta:
                return True

        for (x, y, w, h) in self.obs_boundary:
            if 0 <= coords[0] - (x - delta) <= w + 2 * delta \
                    and 0 <= coords[1] - (y - delta) <= h + 2 * delta:
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


####### USEFUL VARIABLES #########
# all the measurements are in meters
mass = 116  # mass of the robot
a = 0.37  # distance from the front wheel of G
b = 0.55  # distance from the rear wheel of G
mu = 0.895  # lateral friction coefficient
fr = 0.1  # frontal friction coefficient
Iz = 20  # inertia moment of the robot Kgm^2
g = 9.81  # gravity acceleration constant
d0 = 0.18  # 0 < d0 < a
r = 0.2  # wheel radius


## BOUNDS ##
max_vel = 2  # max achievable velocity
tau_max = 1  # Nm
cmd_bd = 12


t = 0.63/2  # width of the robot
kv = 65
kp = 1.5*kv
ka = 0.1*kv
x_bounds = (0, 50)
y_bounds = (0, 30)
freq = 100
prob_gs = 0.1
n_iters = 700
step = 10
######## INITIAL CONDITION ##########
start = 2, 5  # starting node
goal = 45, 5  # goal node

tot_time = math.sqrt((start[0]-goal[0])**2 + (start[1]-goal[1])**2)/(max_vel) + 1

theta = 10*math.pi/180
xd = 0.5  # m/s
yd = 0.5  # m/s
thetad = -2.2  # rad/s
xdd = 0  # m/s^2
ydd = 0  # m/s^2
thetadd = 0  # rad/s^2
u0 = 0  # initial command

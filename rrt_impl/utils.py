import env
import math
import numpy as np
import os
import sys
import cv2

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
        self.x_range = 50
        self.y_range = 30
        self.env = env.Env(x_bounds, y_bounds)

        self.delta = 1
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

    def is_inside_obs_alternative(self, coords, vels):
        delta = self.delta

        x, y = coords
        xd, yd = vels
        samp_x, samp_y = int(round(x*res)), int(round(y*res))
        
        if not samp_x in range(0, self.x_range) or not samp_y in range(0, self.y_range):
            
            return True

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

class Utils_m:

    def __init__(self, map, length, tol, pitch_b, roll_b, res = 1, trav_map = None):

        self.x_range, self.y_range = map.shape
        self.map = map
        self.length = length
        self.tol = tol
        self.res = res
        self.pitch_b = pitch_b
        self.roll_b = roll_b
        self.trav_map = trav_map


        self.x_obs = np.where(self.map != 0)[0]
        self.y_obs = np.where(self.map != 0)[1]

        
    def is_inside_obs_alternative(self, state, vels):

        x, y = state
        xd, yd = vels
        samp_x, samp_y = int(round(x*res)), int(round(y*res))
   
        if abs(xd) > max_vel or abs(yd) > max_skid or abs(xd) < 0.001:
            # print("vels: ", xd, yd, max_skid, xd)
            return True
        
        if not samp_x in range(0, self.x_range) or not samp_y in range(0, self.y_range):
            # print("ranges: ", samp_x, samp_y)

            return True

        length = self.length
        tol = self.tol
        # print("STATE: ", x, y)
        x_bounds = (x-length-tol, x+length+tol)
        y_bounds = (y-length-tol, y+length+tol)



        for obs_idx in range(self.x_obs.shape[0]):
   
            if x_bounds[0]*res <= self.x_obs[obs_idx] <= x_bounds[-1]*res and \
               y_bounds[0]*res <= self.y_obs[obs_idx] <= y_bounds[-1]*res:
                # print('OBSTACLE: ', self.x_obs[obs_idx], self.y_obs[obs_idx])
                # print('Bounds:', (x_bounds[0], x_bounds[1]),(y_bounds[0], y_bounds[1]))
                return True
            
        return False
    
    def is_transition_feasible(self, prev_state, state, vels):

        x, y = state
        x_old, y_old = prev_state
        xd, yd = vels
        # print("prev: ", x, y)
        # print("res: ", x*res, y*res)
        # print("round: ", round(x*res), round(y*res))



        samp_x, samp_y = int(round(x*res)), int(round(y*res))
        samp_x_old, samp_y_old = int(round(x_old*res)), int(round(y_old*res))
        
        if not samp_x in range(0, self.x_range) or not samp_y in range(0, self.y_range) or not samp_x_old in range(0, self.x_range) or not samp_y_old in range(0, self.y_range):
            return True, 0

        z = self.map[samp_x, samp_y]
        z_old = self.map[samp_x_old, samp_y_old]


        if abs(xd) > max_vel or abs(yd) > max_skid:
            print('velocity x not in range: ', xd)

            print('velocity y not in range: ', yd)
            return True, 0
        
        
        # print(samp_x, samp_y)
        if self.trav_map[samp_x, samp_y]:
            # print("obs: ", samp_x, samp_y)

            return True, 0
        
        return False, math.hypot(z-z_old, math.hypot(x-x_old, y-y_old))

        

        
    
def trav_map(old_map, dist, pitch_toll, roll_toll):
    R, C = old_map.shape
    travers_map = np.zeros(old_map.shape)
    for i in range(R):
        for j in range(C):

        
            points = []
            out = []

            for row in range(i, i-dist-1, -1):
                if row < 0:
                    break
                for col in range(j, j-dist-1, -1):
                    if col < 0:
                        break
                    if [1, row, col] not in points:
                        points.append([1, row, col])
                        out.append([old_map[row, col]])


            for row in range(i, i+dist+1):
                if row >= R:
                    break
                for col in range(j, j+dist+1):
                    if col >= C:
                        break
                    if [1, row, col] not in points:
                        points.append([1, row, col])
                        out.append([old_map[row, col]])


            X = np.array(points)
            out = np.array(out)

            # LEAST SQUARES METHOD
            w = np.linalg.pinv(X) @ out

            n = np.array([w[1][0], w[2][0], -1])        
            norm = np.linalg.norm(n)

            w_n = n/norm # Cosine directors.

            nx = w_n[0]
            ny = w_n[1]
            nz = -w_n[2]


            yaw = math.atan2(ny,nx)
            pitch = math.atan2(nx,nz)
            roll = math.atan2(nz,ny)

            pitch_bound = pitch_toll
            roll_bound = roll_toll 

            cond = False

            if pitch_bound <= pitch <= 2*math.pi - pitch_bound:
                    cond = True

            if roll <= - roll_bound + math.pi/2 or roll >= roll_bound + math.pi/2:
                    cond = True

            travers_map[i, j] = cond
    return travers_map



        
def get_new_map(map, map_name):
    d = {'many_obstacles': (400, 300),
         'few_obstacles':  (200, 120),
         'empty': (120, 120),
         'apollo15_landing_site': 514}
    vals = d[map_name]

    if map_name == 'apollo15_landing_site':
        map = cv2.rotate(map[:vals, :], cv2.ROTATE_180) 
        return map

    if map_name == 'empty':
        map[0, 0] = 1

    map = map[:vals[0], :vals[0]]
    map = cv2.rotate(map, cv2.ROTATE_180)
    map = map[:vals[-1], :vals[-1]]
    
    # plt.imshow(1-map, cmap = 'gray')
    return map

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
max_vel = 1  # max achievable velocity
max_skid = 0.1
tau_max = 250  # Nm
cmd_bd = 12

length = 0.5
tol = 0.2


t = 0.63/2  # width of the robot
kv = 1#65
kp = 1.5*kv
ka = 0.1*kv
x_bounds = (0, 50)
y_bounds = (0, 30)
freq = 10
prob_gs = 0.1
n_iters = 2000


pitch_bound = 2*math.pi/180
roll_bound = 2*math.pi/180


jerk = True
elevation = False

map_title = 'many_obstacles.npy'
trav_map = None

if map_title != 'apollo15_landing_site.npy': # ROS: 150-213, -4.3420014 // PYTHON: 150-470
    res = 10
    step = 1000 # we do not consider the step

else:
    res = 2
    elevation = True
    trav_map = np.load('numpy_maps/apollo15_traversability_map.npy')
    step = 10
    start = 50, 500  # apollo15: START (50, 500) --> (100, 50)
    goal = 100, 50

if map_title == 'basic':
    res = 1

######## INITIAL CONDITION ##########
if map_title == 'empty.npy':
    start = 0,0
    goal = 10, 10
    prob_gs = 1
if map_title == 'few_obstacles.npy':
    start = 0,0  
    goal = 10, 10
if map_title == 'many_obstacles.npy':
    start = 0,0  
    goal = 28.38, 28.33
if map_title == 'basic':
    start = 2,2
    goal = 45, 25

obs = True





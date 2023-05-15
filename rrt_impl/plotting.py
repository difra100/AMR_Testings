import env
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")


class Plotting:
    def __init__(self, x_start, x_goal, map = 'basic', el_map = None, res = 1, mode = 'kino'):
        self.xI, self.xG = x_start, x_goal
        x_bounds = (0, 50)
        y_bounds = (0, 30)
        self.map = map
        self.el_map = el_map
        self.res = res
        self.mode = mode
        if map == 'basic':
            self.env = env.Env(x_bounds, y_bounds)
            self.obs_bound = self.env.obs_boundary
            self.obs_circle = self.env.obs_circle
            self.obs_rectangle = self.env.obs_rectangle
        else:
            self.env = map

    def animation(self, nodelist, path, name, animation=False, steer=True, elevation = False):
        
        if self.map != 'basic':
            extent = [0, self.env.shape[1]/self.res, self.env.shape[0]/self.res, 0] # Get correct resolution of the image
        
        if elevation:
            self.plot_grid(name)
            plt.imshow(self.env, cmap = 'gray', extent = extent)
            self.plot_visited(nodelist, animation, steer) # Green Lines code
            self.plot_path(path, steer)

            self.plot_grid(name)
            plt.imshow(1-self.el_map, cmap = 'gray', extent = extent)
            self.plot_visited(nodelist, animation, steer) # Green Lines code
            self.plot_path(path, steer)
        
        elif self.map == 'basic':
            self.plot_grid(name)
            self.plot_visited(nodelist, animation, steer) # Green Lines code
            self.plot_path(path, steer)


        else:
            self.plot_grid(name)
            plt.imshow(1-self.env, cmap = 'gray', extent = extent)
            self.plot_visited(nodelist, animation, steer) # Green Lines code
            self.plot_path(path, steer)
        

    def animation_connect(self, V1, V2, path, name):
        self.plot_grid(name)
        self.plot_visited_connect(V1, V2)
        self.plot_path(path)

    def plot_grid(self, name):
        fig, ax = plt.subplots()

        if self.map == 'basic':
            for (ox, oy, w, h) in self.obs_bound:
                ax.add_patch(
                    patches.Rectangle(
                        (ox, oy), w, h,
                        edgecolor='black',
                        facecolor='black',
                        fill=True
                    )
                )

            for (ox, oy, w, h) in self.obs_rectangle:
                ax.add_patch(
                    patches.Rectangle(
                        (ox, oy), w, h,
                        edgecolor='black',
                        facecolor='gray',
                        fill=True
                    )
                )

            for (ox, oy, r) in self.obs_circle:
                ax.add_patch(
                    patches.Circle(
                        (ox, oy), r,
                        edgecolor='black',
                        facecolor='gray',
                        fill=True
                    )
                )

        if self.map != 'basic':
            plt.plot(self.xI.y, self.xI.x, "cs", linewidth=3)
            plt.plot(self.xG.y, self.xG.x, "ms", linewidth=3)
        else:
            plt.plot(self.xI.x, self.xI.y, "cs", linewidth=3)
            plt.plot(self.xG.x, self.xG.y, "ms", linewidth=3)

        plt.title(name)
        plt.axis("equal")

    
    def plot_visited(self, nodelist, animation, steer):
        if animation:
            count = 0
            for node in nodelist:
                count += 1
                if node.parent:
                    plt.plot([node.parent.x, node.x], [
                             node.parent.y, node.y], "-g")
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])
                    if count % 10 == 0:
                        plt.pause(0.001)
        
        else:
            x_idx = 0
            y_idx = 1
            if self.map != 'basic':
                    x_idx = 1
                    y_idx = 0
            for node in nodelist:
                if node.parent:

                    if steer:
                        plt.scatter([node.confs[i][x_idx][0] for i in range(node.confs.shape[0])], [
                        node.confs[i][y_idx][0] for i in range(node.confs.shape[0])], s=2., c='green')
                        if self.map != 'basic':
                            plt.scatter([node.y], [
                                        node.x], s=3, c='green', marker = 's')
                        else:
                            plt.scatter([node.x], [
                                        node.y], s=3, c='green', marker = 's')
                    else: 
                        plt.scatter(list(np.linspace(node.parent.x, node.x, 100)), list(np.linspace(node.parent.y, node.y, 100)), s=1, c='green', marker = 's')
                
                    

    @staticmethod
    def plot_visited_connect(V1, V2):
        len1, len2 = len(V1), len(V2)

        for k in range(max(len1, len2)):
            if k < len1:
                if V1[k].parent:
                    plt.plot([V1[k].x, V1[k].parent.x], [
                             V1[k].y, V1[k].parent.y], "-g")
            if k < len2:
                if V2[k].parent:
                    plt.plot([V2[k].x, V2[k].parent.x], [
                             V2[k].y, V2[k].parent.y], "-g")

            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            if k % 2 == 0:
                plt.pause(0.001)

        plt.pause(0.01)

    def plot_path(self, path, steer=True):
        if len(path) != 0:
            x_idx = 0
            y_idx = 1
            if self.map != 'basic':
                    x_idx = 1
                    y_idx = 0
            if steer:
                
                plt.scatter([x[0].confs[i][x_idx][0] for x in path for i in range(x[0].confs.shape[0])], [
                    x[0].confs[i][y_idx][0] for x in path for i in range(x[0].confs.shape[0])], s=0.05, c='blue')  # '-r', linewidth=1)
                plt.scatter([x[0].confs[-1][x_idx][0] for x in path], [x[0].confs[-1][y_idx][0]
                                                                for x in path], s=1, c='red', marker = 's')
            else:
                plt.plot([x[0].x for x in path], [x[0].y
                         for x in path], '-r', linewidth=2)

            plt.pause(0.01)
            plt.show()
            
            return

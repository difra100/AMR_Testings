import env
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")


class Plotting:
    def __init__(self, x_start, x_goal):
        self.xI, self.xG = x_start, x_goal
        x_bounds = (0, 50)
        y_bounds = (0, 30)
        self.env = env.Env(x_bounds, y_bounds)
        self.obs_bound = self.env.obs_boundary
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle

    def animation(self, nodelist, path, name, animation=False, steer=True, real=False, steer2=False):
        self.plot_grid(name)
        self.plot_visited(nodelist, animation) # Green Lines code
        self.plot_path(path, steer, real, steer2)

    def animation_connect(self, V1, V2, path, name):
        self.plot_grid(name)
        self.plot_visited_connect(V1, V2)
        self.plot_path(path)

    def plot_grid(self, name):
        fig, ax = plt.subplots()

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

        plt.plot(self.xI.x, self.xI.y, "bs", linewidth=3)
        plt.plot(self.xG.x, self.xG.y, "gs", linewidth=3)

        plt.title(name)
        plt.axis("equal")

    @staticmethod
    def plot_visited(nodelist, animation):
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
            for node in nodelist:
                if node.parent:
                    plt.scatter([node.confs[i][0][0] for i in range(node.confs.shape[0])], [
                    node.confs[i][1][0] for i in range(node.confs.shape[0])], s=0.5, c='green')
                    plt.scatter([node.x], [
                    node.y], s=1, c='green')

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

    @staticmethod
    def plot_path(path, steer=True, real=False, steer2=True):
        if len(path) != 0:
            if not steer and real:
                conf = path
                plt.plot([x[0] for x in conf], [
                         x[1] for x in conf], '-b', linewidth=2)
                return
            if steer:
                plt.scatter([x[0].conf[i][0] for x in path for i in range(len(x[0].conf))], [
                    x[0].conf[i][1] for x in path for i in range(len(x[0].conf))], s=2, c='blue')  # '-r', linewidth=1)
                conf = path
                plt.scatter([x[0].conf[-1][0] for x in conf], [
                    x[0].conf[-1][1] for x in conf], s=30, c='red')
                # plt.plot([k[0].conf[i][0]  for k in path for i in range(len(k[0].conf))], [k[0].conf[i][1]  for k in path for i in range(len(k[0].conf))], '-r', linewidth=2)

                return
            elif steer2:
                plt.scatter([x[0].confs[i][0][0] for x in path for i in range(x[0].confs.shape[0])], [
                    x[0].confs[i][1][0] for x in path for i in range(x[0].confs.shape[0])], s=2, c='blue')  # '-r', linewidth=1)
                plt.scatter([x[0].confs[-1][0] for x in path], [x[0].confs[-1][1]
                                                                for x in path], s=30, c='red')
            else:
                plt.plot([x[0].x for x in path], [x[0].y
                         for x in path], '-r', linewidth=2)

            plt.pause(0.01)
            return
        plt.show()

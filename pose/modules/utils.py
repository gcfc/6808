# TEAM 1 utils.py
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.style as style
import random
import numpy as np

style.use("ggplot")
body_parts = ['nose', 'neck',
              'r_sho', 'r_elb', 'r_wri', 'l_sho', 'l_elb', 'l_wri',
              'r_hip', 'r_knee', 'r_ank', 'l_hip', 'l_knee', 'l_ank',
              'r_eye', 'l_eye',
              'r_ear', 'l_ear']


class Graph():
    def __init__(self, ax, coords, delay):
        self.ax = ax
        # delay: in seconds
        self.delay = delay
        # check if a body part point is missing or not
        self.availability = {}
        # coords should be a dictionary of length 3 lists/tuples
        # each list/tuple should correspond to the correct body part
        self.id_p = {
            body_parts[i]: i for i in range(len(body_parts))
        }

    def plot(self):
        self.ax.clear()
        self.ax.scatter(self.x, self.y, self.z, s=100, label="Body Pose Position")
        self.draw()

    def draw(self):
        self.trace("nose", "neck")

        self.trace("neck", "r_sho")
        self.trace("r_sho", "r_elb")
        self.trace("r_elb", "r_wri")

        self.trace("neck", "r_hip")
        self.trace("r_hip", "r_knee")
        self.trace("r_knee", "r_ank")

        self.trace("neck", "l_sho")
        self.trace("l_sho", "l_elb")
        self.trace("l_elb", "l_wri")

        self.trace("neck", "l_hip")
        self.trace("l_hip", "l_knee")
        self.trace("l_knee", "l_ank")

    # coords is a dictionary pointing body_part_id to (x,y,z)
    def update_coords(self, coords):
        lists = []
        for i in range(len(body_parts)):
            if i not in coords:
                lists.append([0, 0, 0])
                self.availability[i] = False
            else:
                lists.append(coords[i])
                self.availability[i] = True
        lists = list(zip(*lists))
        self.x = np.array(lists[0], dtype=np.float)
        self.y = np.array(lists[1], dtype=np.float)
        self.z = np.array(lists[2], dtype=np.float)

    def trace(self, part1, part2):
        if not self.availability[self.id_p[part1]] or not self.availability[self.id_p[part2]]:
            return

        self.ax.plot3D([self.x[self.id_p[part1]], self.x[self.id_p[part2]]],
                [self.y[self.id_p[part1]], self.y[self.id_p[part2]]],
                [self.z[self.id_p[part1]], self.z[self.id_p[part2]]], 'b')

    def show(self):
        plt.show(block=False)
        plt.pause(self.delay)
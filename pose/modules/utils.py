# TEAM 1 utils.py
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.style as style

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
        # coords should be a list of length 3 lists
        # each sublist should correspond to the correct body part
        self.x, self.y, self.z = self.update_coords(coords)
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

    def update_coords(self, coords):
        x, y, z = [], [], []
        if len(coords) == 0:
            return x,y,z
        if len(coords[0]) != 3:
            print("Coords must be length 3")
            return x,y,z
        coords = list(zip(*coords))
        x = np.array(coords[0], dtype=np.float)
        y = np.array(coords[1], dtype=np.float)
        z = np.array(coords[2], dtype=np.float)
        return x,y,z

    def trace(self, part1, part2):
        self.ax.plot3D([self.x[self.id_p[part1]], self.x[self.id_p[part2]],
                self.y[self.id_p[part1]], self.y[self.id_p[part2]],
                self.z[self.id_p[part1]], self.z[self.id_p[part2]]])

    def show(self):
        plt.show(block=False)
        plt.pause(self.delay)
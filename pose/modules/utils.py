# TEAM 1 utils.py
import tkinter
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
    def __init__(self, fig, ax, coords, delay):
        # max length of axis
        MAX_LENGTH = 2000
        # instantiate plot
        ax.set_xlim(left=0, right=MAX_LENGTH)
        ax.set_ylim(bottom=0, top=MAX_LENGTH)
        ax.set_zlim(bottom=-MAX_LENGTH, top=MAX_LENGTH)
        ax.view_init(elev=90., azim=90)
        plt.show(block=False)
        plt.pause(0.1)

        # class variables for plot
        self.fig = fig
        self.ax = ax
        self.coords = coords

        # delay: in seconds
        self.delay = delay

        # coords should be a dictionary of length 3 lists/tuples
        # each list/tuple should correspond to the correct body part
        self.id_p = {
            body_parts[i]: i for i in range(len(body_parts))
        }

        # lines in graph
        self.lines = {}

        # mapping of whether a body_id is available for a time frame
        self.available = {i: False for i in range(len(body_parts))}

        # cached background for matplotlib blit
        self.background = fig.canvas.copy_from_bbox(fig.bbox)

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
    # deltas is a dictionary pointing body_part_id to (dx,dy,dz)
    def update_coords(self, coords={}, deltas={}, dims=3, override=False):
        if override:
            for i in range(len(body_parts)):
                self.available[i] = i in coords
            self.coords = coords
        else:
            for body_id in range(len(body_parts)):
                if not self.available[body_id]: return
                # update x,y,z
                for i in range(dims):
                    self.coords[body_id][i] += deltas[body_id][i]


    def trace(self, part1, part2):
        label1 = self.id_p[part1]
        label2 = self.id_p[part2]
        line_label = (label1, label2)

        if not self.available[label1] or not self.available[label2]:
            if line_label in self.lines:
                self.lines[line_label].set_data_3d([0, 0], [0, 0], [0, 0])
            return

        x1, y1, z1 = self.coords[label1]
        x2, y2, z2 = self.coords[label2]
        if line_label not in self.lines:
            
            (line,) = self.ax.plot3D([x1, x2], [y1, y2], [z1, z2], 'b')
            self.lines[line_label] = line
        else:
            self.lines[line_label].set_data_3d([x1, x2], [y1, y2], [z1, z2])

    def plot(self):
        self.fig.canvas.restore_region(self.background)
        for line in self.lines.values():
            self.ax.draw_artist(line)
        self.fig.canvas.blit(self.fig.bbox)
        self.fig.canvas.flush_events()
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

DIMENSIONS = 3

# max length of axis
MAX_LENGTH = 2000

class Graph():
    def __init__(self, fig, ax, coords):
        # instantiate plot
        ax.set_xlim(left=0, right=MAX_LENGTH)
        ax.set_ylim(bottom=0, top=MAX_LENGTH)
        ax.set_zlim(bottom=-MAX_LENGTH, top=MAX_LENGTH)
        ax.set_xlabel("X-axis")
        ax.set_ylabel("Y-axis")
        ax.set_zlabel("Z-axis")
        ax.view_init(elev=120., azim=-90)
        plt.show(block=False)
        plt.pause(0.1)

        # class variables for plot
        self.fig = fig
        self.ax = ax
        self.coords = coords

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

        # cache for previous imu coords
        self.prev_imu_coords = None

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
    def update_coords(self, cv_coords={}, imu_coords={}, override=False):
        if override:
            for i in range(len(body_parts)):
                if i in cv_coords:
                    self.coords[i] = cv_coords[i]
                    self.coords[i][0] = MAX_LENGTH-self.coords[i][0]
                    self.coords[i][1] = MAX_LENGTH-self.coords[i][1]
                self.available[i] = i in self.coords
            # self.coords = cv_coords
        else:
            deltas = self.calc_deltas(imu_coords)
            for body_id in range(len(body_parts)):
                if not self.available[body_id]: continue
                # update x,y,z
                for i in range(DIMENSIONS):
                    self.coords[body_id][i] += deltas[body_id][i]
            # print(deltas)

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

    def get_gesture(self):
        pass

    def calc_deltas(self, imu_coords):
        if self.prev_imu_coords is None:
            self.prev_imu_coords = imu_coords
            return {i:[0,0,0] for i in range(len(body_parts))}

        deltas = {i: [0,0,0] for i in range(len(body_parts))}
        for body_part_id in range(len(body_parts)):
            if (body_part_id in [2,3,4]): # only doing arm temporarily
                for i in range(DIMENSIONS):
                    # need to multiply -1 on x-y for now because 0,0 is at top right corner
                    # length from elbow to wrist: 25 cm; shoulder to elbow: 22.5
                    px_over_m = ((945-666)**2 + (784-767)**2)**.5 / 0.225
                    deltas[body_part_id][i] = px_over_m * (imu_coords[body_part_id][i] - self.prev_imu_coords[body_part_id][i])
        self.prev_imu_coords = imu_coords
        return deltas

# helper to calculate imu coords from arduino
def calc_imu_coords(line):
    line = list(map(float, line))
    return {
        2: [0,0,0],
        3: [-line[2], -line[1], line[3]],
        4: [-line[5] - line[2], -line[4] - line[1], line[6]]
    }
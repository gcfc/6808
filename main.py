import argparse

import cv2
import numpy as np
import torch
import time
import matplotlib.pyplot as plt

from pose.models.with_mobilenet import PoseEstimationWithMobileNet
from pose.modules.load_state import load_state
from pose.modules.utils import Graph
from pose.demo import run_demo

### NOTES FOR TEAM 1 ###

## INSTRUCTIONS

# (1) Please download Daniil-Osokin's checkpoint_iter_370000.pth and add it under models folder

# (2) All comments elsewhere besides this file must NOT be deleted without every team member's approval
# (3) Lines of code that are added by TEAM 1 in other files MUST be marked with "#TEAM" at the end of the line
# (4) For any other function that you will define and use in main(), please implement in modules/utils.py and import the function here

## HOW TO START PROGRAM

# If you would like to test with VIDEO:
# python main.py --checkpoint-path <path_to>/checkpoint_iter_370000.pth --video 0
# example: python main.py --checkpoint-path ./models/checkpoint_iter_370000.pth --video 0

# If you would like to test with IMAGES:
# python main.py --checkpoint-path <path_to>/checkpoint_iter_370000.pth --images <path_to_image>
# example: python main.py --checkpoint-path ./models/checkpoint_iter_370000.pth --images images/test_img.jpg


## REFERENCE

# The following are supported joints from ML. The indx of an item is its body part id
# kpt_names = ['nose', 'neck',
#                 'r_sho', 'r_elb', 'r_wri', 'l_sho', 'l_elb', 'l_wri',
#                 'r_hip', 'r_knee', 'r_ank', 'l_hip', 'l_knee', 'l_ank',
#                 'r_eye', 'l_eye',
#                 'r_ear', 'l_ear']

def main():
    #graph is also delayed by the same value, so 2*delay is the actual delay time. why twice: must delay graph or else pyplot freezes
    delay = 0.001

    # variable for video camera
    cam = None

    #initializing graph
    fig = plt.figure()
    map_ax = fig.add_subplot(111, projection="3d")
    map_ax.autoscale(enable=True, axis='both', tight=True)
    graph = Graph(ax=map_ax, coords=[], delay=delay)

    while True:
        # TODO get data from IMUs

        # pass webcam pic into analyzer to get body pose points
        if args.video != '':
            if cam is None:
                cam = cv2.VideoCapture(args.video)
            was_read, img = cam.read()
            if not was_read or not cam.isOpened():
                print("Camera is either closed or missing.")
                break
        else:
            # only analyze first image of given list of images
            img = cv2.imread(args.images[0], cv2.IMREAD_COLOR)
        frame_provider = [img]
        points = run_demo(net, frame_provider, args.height_size, args.cpu, args.track, args.smooth)

        # TODO invoke utility functions to calculate final body pose points

        # plot calculated points on animated 3D graph
        # graph.update_coords(coords)
        # graph.plot()
        # graph.show()

        time.sleep(delay)


# DO NOT EDIT THE FOLLOWING CODE, EDIT ONLY IN main()
if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='''Team 1 Body Pose Tracking with IMUs''')
    parser.add_argument('--checkpoint-path', type=str, required=True, help='path to the checkpoint')
    parser.add_argument('--height-size', type=int, default=256, help='network input layer height size')
    parser.add_argument('--video', type=str, default='', help='path to video file or camera id')
    parser.add_argument('--images', nargs='+', default='', help='path to input image(s)')
    parser.add_argument('--cpu', action='store_true', help='run network inference on cpu')
    parser.add_argument('--track', type=int, default=1, help='track pose id in video')
    parser.add_argument('--smooth', type=int, default=1, help='smooth pose keypoints')
    args = parser.parse_args()

    net = PoseEstimationWithMobileNet()
    checkpoint = torch.load(args.checkpoint_path, map_location='cpu')
    load_state(net, checkpoint)

    main()


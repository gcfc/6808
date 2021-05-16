import argparse

import signal
import sys
import threading
import serial
import cv2
import numpy as np
import torch
import time
import os
import tkinter
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import requests # tmp to get img from webcam
from collections import deque

from pose.models.with_mobilenet import PoseEstimationWithMobileNet
from pose.modules.load_state import load_state
from pose.modules.utils import Graph, calc_imu_coords
from pose.demo import run_demo

from depth.networks.resnet_encoder import ResnetEncoder
from depth.networks.depth_decoder import DepthDecoder
from depth.test_simple import run_depth

### NOTES FOR TEAM 1 ###

## INSTRUCTIONS

# (1) Please download Daniil-Osokin's checkpoint_iter_370000.pth and add it under models folder

# (2) All comments elsewhere besides this file must NOT be deleted without every team member's approval
# (3) Lines of code that are added by TEAM 1 in other files MUST be marked with "#TEAM" at the end of the line
# (4) For any other function that you will define and use in main(), please implement in modules/utils.py and import the function here

## HOW TO START PROGRAM

# If you would like to test with VIDEO:
# python main.py --checkpoint-path <path_to>/checkpoint_iter_370000.pth --video 0
# example: python main.py --checkpoint-path ./pose/models/checkpoint_iter_370000.pth --video 0

# If you would like to test with IMAGES:
# python main.py --checkpoint-path <path_to>/checkpoint_iter_370000.pth --images <path_to_image>
# example: python main.py --checkpoint-path ./pose/models/checkpoint_iter_370000.pth --images images/test_img.jpg


## REFERENCE

# The following are supported joints from ML. The indx of an item is its body part id
# kpt_names = ['nose', 'neck',
#                 'r_sho', 'r_elb', 'r_wri', 'l_sho', 'l_elb', 'l_wri',
#                 'r_hip', 'r_knee', 'r_ank', 'l_hip', 'l_knee', 'l_ank',
#                 'r_eye', 'l_eye',
#                 'r_ear', 'l_ear']

def signal_handler(sig, frame):
    print('You pressed Ctrl+C! Ending all threads...')
    sys.exit(0)

def run_cv():
    if img is None: return
    cv_coords = run_demo(net, [img], args.height_size, args.cpu, args.track, args.smooth)
    depth = run_depth(encoder, depth_decoder, feed_width, feed_height, device, [img])
    buffer.append(cv_coords)

def cv_process():
    while True:
        run_cv()
        time.sleep(timeout)

def main():
    global cam, img, buffer

    # flags to see if initialization is complete
    imu_ready = False
    cv_ready = False

    # initializing graph
    fig = plt.figure()
    map_ax = fig.add_subplot(111, projection="3d")
    map_ax.autoscale(enable=True, axis='both', tight=True)
    graph = Graph(fig=fig, ax=map_ax, coords={})

    # intialize arduino
    arduino = serial.Serial(port='COM8', baudrate=115200, timeout=.1)

    # initialize camera
    if args.video != '':
        if cam is None:
            args.video = int(args.video) if args.video.isnumeric() else args.video
            cam = cv2.VideoCapture(args.video)
    else:
        # assuming an image is provided
        img = cv2.imread(args.images[0], cv2.IMREAD_COLOR)

    # start asynchronous machine learning task
    t1 = threading.Thread(target=cv_process, args=(), daemon=True)
    t1.start()
    
    while True:
        # get data from IMUs
        try:
            line = arduino.readline().decode().rstrip().split()
        except UnicodeDecodeError: continue
        except KeyboardInterrupt:   break
        if not imu_ready:
            imu_ready = "VALIDATED!" in line
            continue
        imu_coords = calc_imu_coords(line)

        # pass webcam pic into analyzer to get body pose points
        # local cam
        was_read, img = cam.read()
        if not was_read or not cam.isOpened():
            print("Camera is either closed or missing.")
            return

        # online webcam
        # r = requests.get("http:///shot.jpg")
        # img_arr = np.array(bytearray(r.content), dtype=np.uint8)
        # img = cv2.imdecode(img_arr, -1)

        # invoke machine learning models to retrieve body pose points
        if not cv_ready:
            run_cv()
            cv_ready = True
            continue

        cv_coords = buffer.popleft() if len(buffer) else None

        # plot calculated points on animated 3D graph 
        graph.update_coords(cv_coords=cv_coords, imu_coords=imu_coords, override=cv_coords is not None)
        graph.draw()
        graph.plot()


# DO NOT EDIT THE FOLLOWING CODE, EDIT ONLY IN main()
if __name__ == '__main__':
    # listener for ctrl + c to exit process
    signal.signal(signal.SIGINT, signal_handler)

    # initializing par
    parser = argparse.ArgumentParser(
        description='''Team 1 Body Pose Tracking with IMUs''')

    # parser for command line arguments
    parser.add_argument('--checkpoint-path', type=str, required=True, help='path to the checkpoint')
    parser.add_argument('--height-size', type=int, default=256, help='network input layer height size')
    parser.add_argument('--video', type=str, default='', help='path to video file or camera id')
    parser.add_argument('--images', nargs='+', default='', help='path to input image(s)')
    parser.add_argument('--cpu', action='store_true', help='run network inference on cpu')
    parser.add_argument('--track', type=int, default=1, help='track pose id in video')
    parser.add_argument('--smooth', type=int, default=1, help='smooth pose keypoints')

    # parser for depth
    parser.add_argument('--model_name', type=str,
                        help='name of a pretrained model to use',
                        choices=[
                            "mono_640x192",
                            "stereo_640x192",
                            "mono+stereo_640x192",
                            "mono_no_pt_640x192",
                            "stereo_no_pt_640x192",
                            "mono+stereo_no_pt_640x192",
                            "mono_1024x320",
                            "stereo_1024x320",
                            "mono+stereo_1024x320"], default="mono+stereo_640x192")
    parser.add_argument('--ext', type=str,
                        help='image extension to search for in folder', default="jpg")
    parser.add_argument("--no_cuda",
                        help='if set, disables CUDA',
                        action='store_true')

    args = parser.parse_args()

    # initializing pose model
    net = PoseEstimationWithMobileNet()
    checkpoint = torch.load(args.checkpoint_path, map_location='cpu')
    load_state(net, checkpoint)

    # initializing depth model
    if torch.cuda.is_available() and not args.no_cuda:
        device = torch.device("cuda")
    else:
        device = torch.device("cpu")

    model_path = "./depth/models/mono+stereo_640x192"
    # print("-> Loading model from ", model_path)
    encoder_path = model_path + "/encoder.pth"
    depth_decoder_path = model_path + "/depth.pth"

    # LOADING PRETRAINED MODEL
    # print("   Loading pretrained encoder")
    encoder = ResnetEncoder(18, False)
    loaded_dict_enc = torch.load(encoder_path, map_location=device)

    # extract the height and width of image that this model was trained with
    feed_height = loaded_dict_enc['height']
    feed_width = loaded_dict_enc['width']
    filtered_dict_enc = {k: v for k, v in loaded_dict_enc.items() if k in encoder.state_dict()}
    encoder.load_state_dict(filtered_dict_enc)
    encoder.to(device)
    encoder.eval()

    # print("   Loading pretrained decoder")
    depth_decoder = DepthDecoder(
        num_ch_enc=encoder.num_ch_enc, scales=range(4))

    loaded_dict = torch.load(depth_decoder_path, map_location=device)
    depth_decoder.load_state_dict(loaded_dict)

    depth_decoder.to(device)
    depth_decoder.eval()

    # variable for video camera
    cam = None

    # img of latest camera frame
    img = None

    # buffer of body points shared by every thread
    buffer = deque([])

    # timeout before invoking machine learning models
    timeout = 0.1

    # listen if ctrl+c is invoked
    main()


from __future__ import print_function
import numpy as np
import cv2
import time
import colorsys
import copy
import threading
from collections import deque


class Timer(object):

    def __init__(self, name):
        self.name = name

    def __enter__(self):
        self.start = time.time()

    def __exit__(self, type, value, traceback):
        elapsed = (time.time() - self.start) * 1000
        print("%s took %0.2f ms" % (self.name, elapsed))


class FrameProcessor(object):

    def __init__(self):
        self.n = 200
        self.past_frames = deque([], maxlen=self.n)
        self.windowed_mean = None
        self.windowed_stdev = None
        self.ema = None
        self.last_frame = None
        self.frame_count = 0

        self.worker = threading.Thread(target=self.compute_means)
        self.worker.daemon = True
        # self.worker.start()

        self.fgbg = cv2.createBackgroundSubtractorMOG2()

    def compute_means(self):
        while True:
            if len(self.past_frames) < self.n:
                time.sleep(0.05)
                continue

            with Timer("new values"):
                past_copy = np.array(self.past_frames).copy()
                self.windowed_mean = np.mean(past_copy, axis=0)
                self.windowed_stdev = np.std(past_copy, axis=0)

    def get_ping_foreground(self, frame):
        frame = frame.astype(np.float32)
        self.last_frame = frame
        self.past_frames.append(frame)
        self.frame_count += 1

        if self.frame_count < self.n:
            return np.zeros(frame.shape, dtype=np.float32)

        if self.ema is None:
            self.ema = np.zeros(frame.shape, dtype=np.float32)

        with Timer("core"):
            v = (frame + self.last_frame) * 0.5
            dev = np.abs(v - self.windowed_mean) / \
                np.clip(self.windowed_stdev, 255 * 0.02, 255)
            signal = np.abs(dev - self.ema)
            signal[dev <= 1] = 0

            self.ema *= 0.9
            self.ema += (dev * 0.1)

            out = signal * 60
            out[signal < 0] = 0
            out[out > 255] = 255

        return out

    def process(self, frame):
        fg = self.fgbg.apply(frame)
        fg = cv2.blur(fg, (5, 5))
        fg[fg > 0] = 255

        params = cv2.SimpleBlobDetector_Params()
        params.filterByCircularity = False
        params.filterByConvexity = False
        params.minDistBetweenBlobs = 20
        params.filterByArea = True
        params.minArea = 50
        params.maxArea = 1000
        params.filterByColor = False
        params.filterByInertia = False
        params.blobColor = 255

        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(fg)

        in_color = cv2.cvtColor(fg, cv2.COLOR_GRAY2RGB)
        im_with_keypoints = cv2.drawKeypoints(in_color, keypoints, np.array(
            []), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        return im_with_keypoints

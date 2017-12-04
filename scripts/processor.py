from __future__ import print_function
import numpy as np
import cv2
import time
import colorsys
import copy
import threading
import OSC
import sys
import json
import codecs
import rospy
from collections import deque
from geometry_msgs.msg import PoseArray, Pose, Point
from sort import Sort


class Timer(object):

    def __init__(self, name):
        self.name = name

    def __enter__(self):
        self.start = time.time()

    def __exit__(self, type, value, traceback):
        elapsed = (time.time() - self.start) * 1000
        print("%s took %0.2f ms" % (self.name, elapsed))


M = 255

class FrameProcessor(object):

    def __init__(self, inputs):
        with open(inputs["config"]) as f:
            self.config = json.load(f)
            self.ping_mode = self.config["cvTuning"]["usePingBGSubtraction"]

        self.n = self.config["cvTuning"]["windowFrames"]
        self.past_frames = deque([], maxlen=self.n)
        self.windowed_mean = None
        self.windowed_stdev = None
        self.ema = None
        self.last_frame = None
        self.frame_count = 0
        self.inputs = inputs
        self.start_time = time.time()

        self.fgbg = cv2.createBackgroundSubtractorMOG2(detectShadows=False)

        with codecs.open(self.inputs["transform"], "r", encoding="utf-8-sig") as f:
            self.persp_transform = json.load(f)["coefficients"]

        self.osc = OSC.OSCClient()
        self.osc.connect((self.config["osc"]["host"], self.config["osc"]["port"]))

        if self.ping_mode:
            self.worker = threading.Thread(target=self.compute_means)
            self.worker.daemon = True
            self.worker.start()
            print("STARTING")

        self.tracker = Sort()

    def compute_means(self):
        while True:
            if len(self.past_frames) == 0:
                time.sleep(0.05)
                continue

            with Timer("new values"):
                past_copy = np.array(self.past_frames).copy()
                stdev = np.std(past_copy, axis=0)
                mean = np.mean(past_copy, axis=0)
                self.windowed_stdev = stdev
                self.windowed_mean = mean

    def get_ping_foreground(self, frame, tuning):
        frame = frame.astype(np.float32)
        self.last_frame = frame
        self.past_frames.append(frame)
        self.frame_count += 1

        if self.windowed_mean is None:
            return

        if self.frame_count < self.n:
            return np.zeros(frame.shape, dtype=np.float32)

        if self.ema is None:
            self.ema = np.zeros(frame.shape, dtype=np.float32)

        with Timer("core"):
            v = (frame + self.last_frame) * 0.5
            dev = np.clip(v - self.windowed_mean, 0, M) / \
                np.clip(self.windowed_stdev, M * tuning["noiseSuppression"], M)
            #signal = np.abs(dev - self.ema)
            #signal[dev <= 1] = 0
            signal = dev

            self.ema *= 0.5
            self.ema += (dev * 0.5)

            out = signal * tuning["gain"]
            out[signal < 0] = 0
            out[out > M] = M

        return out

    def photo_to_world(self, x, y):
        [a, b, c, d, e, f, g, h] = self.persp_transform
        divisor = g*x + h*y + 1
        world_x = (a*x + b*y + c) / divisor
        world_y = (d*x + e*y + f) / divisor
        return world_x, world_y

    def process(self, frame):
        with open(self.inputs["config"]) as f:
            cfg = json.load(f)
            tuning = cfg["cvTuning"]

        mat = np.array([
            [1, 0, frame.shape[1]/2],
            [0, 1, frame.shape[0]/2],
            [0, 0, 1]
        ])
        dist = np.array(cfg["distortion"])
        undistorted = cv2.undistort(frame, mat, dist)
        out_image = self.inputs["bridge"].cv2_to_imgmsg(undistorted, "mono8")
        self.inputs["debug"].publish(out_image)

        frame = undistorted

        blur = tuning["blur"]
        if self.ping_mode:
            fg = self.get_ping_foreground(frame, tuning)
        else:
            fg = self.fgbg.apply(frame)

        if self.ping_mode and self.windowed_mean is None:
            return

        fg = fg.astype(np.uint8)

        fgcopy = fg.copy()
        in_color_copy = cv2.cvtColor(fgcopy, cv2.COLOR_GRAY2RGB)
        # return in_color_copy


        # kernel = np.ones((1, 1), np.uint8)
        # fg = cv2.erode(fg, kernel, iterations=1)

        # fg = cv2.blur(fg, (blur, blur))
        # fg = cv2.blur(fg, (blur, blur))


        # fg = cv2.blur(fg, (blur, blur))
        # fg = cv2.blur(fg, (blur, blur))
        if tuning["threshold"]:
            fg[fg > 0] = 255

        in_color = cv2.cvtColor(fg, cv2.COLOR_GRAY2RGB)

                    
        params = cv2.SimpleBlobDetector_Params()
        params.filterByCircularity = False
        params.filterByConvexity = False
        params.minDistBetweenBlobs = tuning["minDistBetweenBlobs"]
        params.filterByArea = True
        params.minArea = tuning["minArea"]
        params.maxArea = tuning["maxArea"]
        params.filterByColor = False
        params.filterByInertia = False
        params.blobColor = 255

        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(fg)
        # keypoints = detector.detect(frame)
        # in_color = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)

        # rospy.loginfo(keypoints[0])

        ar = PoseArray()
        ar.header.frame_id = "/map"
        ar.header.stamp = rospy.Time.now()

        # timestamp in ms
        now = time.time()
        millis = int((now - self.start_time) * 1000)
        if millis > 0x7fffffff:
            # OSC integers are limited to 32 bits, so once every ~24 days
            # we have to reset the timestamp counter to zero.
            self.start_time = now
            millis = 0

        oscmsg = OSC.OSCMessage()
        oscmsg.setAddress("/blobs")
        oscmsg.append(self.config["osc"]["sourceId"])
        oscmsg.append(millis)
        oscmsg.append(len(keypoints))

        track_list = list()

        im_with_keypoints = cv2.drawKeypoints(in_color, keypoints, np.array(
            []), (255, 0, 0), cv2.DRAW_MATCHES_FLAGS_DEFAULT)

        # cv2.putText(
        #     im_with_keypoints,
        #     "2.3, 5.0",
        #     (50, 50),
        #     cv2.FONT_HERSHEY_SIMPLEX,
        #     0.3,
        #     (255, 0, 0)
        # )

        for i, k in enumerate(keypoints):
            x, y = self.photo_to_world(k.pt[0], k.pt[1])
            size = 1 # TODO
            oscmsg.append(x)
            oscmsg.append(y)
            oscmsg.append(size)

            pose = Pose()
            pose.position = Point(x, y, 0)
            ar.poses.append(pose)

            track_list.append([x, y, x + 1, y + 1, 1])

            cv2.putText(
                im_with_keypoints,
                "%d, %d" % (x, y),
                (int(k.pt[0] + 4), int(k.pt[1] - 4)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.32,
                (255, 0, 0),
            )



        if self.config["osc"]["send"]:
            self.osc.send(oscmsg)

        self.inputs["pose_pub"].publish(ar)


        tracks = self.tracker.update(np.array(track_list))

        rospy.loginfo("N TRACKS %d", len(tracks))


	#print(im_with_keypoints.dtype)


        return im_with_keypoints

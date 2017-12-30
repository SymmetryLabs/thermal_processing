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
import colorsys
from threading import Thread
from collections import deque
from geometry_msgs.msg import PoseArray, Pose, Point
from tracker import Tracker



class Timer(object):

    def __init__(self, name):
        self.name = name

    def __enter__(self):
        self.start = time.time()

    def __exit__(self, type, value, traceback):
        elapsed = (time.time() - self.start) * 1000
        if elapsed > 30:
            rospy.loginfo("%s took %0.2f ms" % (self.name, elapsed))



class FrameProcessor(object):

    def __init__(self, inputs):
        with open(inputs["config"]) as f:
            self.config = json.load(f)
            self.ping_mode = self.config["cvTuning"]["usePingBGSubtraction"]

        self.n = self.config["cvTuning"]["windowFrames"]
        self.history = deque()
        self.sq_error_history = deque()
        self.windowed_mean = None
        self.windowed_stdev = None
        self.ema = None
        self.last_frame = None
        self.frame_count = 0
        self.inputs = inputs
        self.start_time = time.time()
        self.M = 255
        self.osc_client = None

        self.hot_mask = cv2.imread(self.inputs["mask"])[:, :, 2]
        self.hot_mask[self.hot_mask < 255] = 0
        

        self.fgbg = cv2.createBackgroundSubtractorMOG2(detectShadows=False)

        with codecs.open(self.inputs["transform"], "r", encoding="utf-8-sig") as f:
            self.persp_transform = json.load(f)["coefficients"]

        self.tracker = Tracker(self.config["tracker"])



    def update_means(self, frame):
        self.history.append(frame)
        self.running_sum += frame

        if len(self.history) >= (self.n + 1):
            expired_value = self.history.popleft()
            self.running_sum -= expired_value

        self.windowed_mean = self.running_sum / len(self.history)

        new_sq_error = np.square(frame - self.windowed_mean)
        self.sq_error_history.append(new_sq_error)
        self.error_sum += new_sq_error

        if len(self.sq_error_history) >= (self.n + 1):
            expired_sq_error = self.sq_error_history.popleft()
            self.error_sum -= expired_sq_error
        self.running_stdev = np.sqrt(self.error_sum / len(self.history))


    def get_ping_foreground(self, frame, tuning):
        frame = frame.astype(np.float32)

        if self.last_frame is None:
            self.last_frame = frame
            return np.zeros_like(frame)

        self.frame_count += 1

        if self.windowed_mean is None:
            self.running_sum = np.zeros(frame.shape, dtype=np.float32)
            self.error_sum = np.zeros(frame.shape, dtype=np.float32)
            self.windowed_mean = np.zeros(frame.shape, dtype=np.float32)
            self.windowed_stdev = np.zeros(frame.shape, dtype=np.float32)
            self.ema = np.zeros(frame.shape, dtype=np.float32)

        if (self.frame_count % tuning["frameSample"]) == 0:
            self.update_means(frame)

        with Timer("core"):
            v = (frame + self.last_frame) * 0.5
            divisor = np.clip(self.windowed_stdev, self.M * tuning["noiseSuppression"], self.M)
            clipped = np.clip(v - self.windowed_mean, 0, self.M)
            # absed = np.clip(np.abs(v - self.windowed_mean), 0, self.M)
            # clipped[self.hot_mask == 255] = absed[self.hot_mask == 255] 


            dev = clipped / divisor
            #signal = np.abs(dev - self.ema)
            #signal[dev <= 1] = 0
            signal = dev

            self.ema *= 0.5
            self.ema += (dev * 0.5)

            out = signal * tuning["gain"]
            out[signal < 0] = 0
            out[out > self.M] = self.M

        self.last_frame = frame

        return out

    def photo_to_world(self, x, y):
        [a, b, c, d, e, f, g, h] = self.persp_transform
        divisor = g*x + h*y + 1
        world_x = (a*x + b*y + c) / divisor
        world_y = (d*x + e*y + f) / divisor
        return world_x, world_y

    def draw_points(self, im, points, color, rad = 2, solid=False):
        for pt in points:
            cv2.circle(im, tuple(map(int, pt)), rad, color, -1 if solid else 1)

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
        # out_image = self.inputs["bridge"].cv2_to_imgmsg(self.hot_mask, "mono8")
        self.inputs["debug"].publish(out_image)

        frame = undistorted

        blur = tuning["blur"]
        if self.ping_mode:
            fg = self.get_ping_foreground(frame, tuning)
        else:
            fg = self.fgbg.apply(frame)

        if self.frame_count < 10:
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
        with Timer("blobs"):
            keypoints = detector.detect(fg)

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

        track_list = list()

        # im_with_keypoints = cv2.drawKeypoints(in_color, keypoints, np.array(
            # []), (255, 0, 0), cv2.DRAW_MATCHES_FLAGS_DEFAULT)

        keypoints = map(lambda k: k.pt, keypoints)
        tracks, ids = self.tracker.update(keypoints)

        oscmsg.append(len(tracks))


        self.draw_points(in_color, keypoints, (255, 0, 0), solid=True)
        self.draw_points(in_color, tracks, (0, 0, 255), rad=4)

        for i, k in enumerate(tracks):
            x, y = self.photo_to_world(*k)
            size = 1 # TODO
            oscmsg.append(ids[i])
            oscmsg.append(x)
            oscmsg.append(y)
            oscmsg.append(size)

            pose = Pose()
            pose.position = Point(x, y, 0)
            ar.poses.append(pose)

            cv2.putText(
                in_color,
                "%s %d, %d" % (ids[i], x, y),
                (int(k[0] + 4), int(k[1] - 4)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.32,
                (0, 0, 255),
            )

        # ntracks = self.tracker.update(np.array(track_list))
        # rospy.loginfo("TRACKER %d %d", len(keypoints), len(ntracks))



        osc_config = cfg.get('osc', {})
        if osc_config.get('send'):
            if not self.osc_client:
                try:
                    client = OSC.OSCClient()
                    client.connect((osc_config["host"], osc_config["port"]))
                except:
                    sys.stderr.write('could not connect: %r\n' % osc_config)
                else:
                    self.osc_client = client
            if self.osc_client:
                #sys.stderr.write('send %r\n' % oscmsg)
                self.osc_client.send(oscmsg)
        else:
            self.osc_client = None

        if osc_config.get('record'):
            with open(osc_config['record'], 'a') as file:
                #sys.stderr.write('record %r\n' % oscmsg)
                file.write(json.dumps(list(oscmsg)) + '\n')

        self.inputs["pose_pub"].publish(ar)

        # ret, thresh = cv2.threshold(fg, 127, 255, 0)
        # im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # for i, c in enumerate(contours):
        #     color = tuple(map(lambda x: x * 255, colorsys.hsv_to_rgb(float(i + 1) * 360.0 / len(contours), 1, 1)))
        #     rospy.loginfo("NEAT %s", str(color))
        #     cv2.drawContours(in_color, [c], 0, color, 1)



        return in_color

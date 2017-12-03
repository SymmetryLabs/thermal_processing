#!/usr/bin/env python
from __future__ import print_function
import rospy
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from processor import FrameProcessor

# saves the latest image from the camera
# usage: python snapshot.py outfile.png

class Node(object):
    def __init__(self):
        rospy.init_node("snapshot", anonymous=True)

        self.image_sub = rospy.Subscriber("/thermal/image_raw", Image, self.callback)
        self.bridge = CvBridge()

    	rospy.spin()

    def callback(self, data):
    	cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
    	filename = sys.argv[1]
    	cv2.imwrite(filename, cv_image)
        print("Saved to %s, ignore the following errors" % filename)
        rospy.signal_shutdown("Saved to %s" % filename)

if __name__ == "__main__":
    node = Node()

#!/usr/bin/env python
import rospy
import json
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from processor import FrameProcessor

class Node(object):
    def __init__(self):
        rospy.init_node("thermal_viz", anonymous=True)

        self.image_pub = rospy.Publisher("viz", Image, queue_size=1000)
        self.image_sub = rospy.Subscriber("image_raw", Image, self.callback)
        self.bridge = CvBridge()

        self.fp = FrameProcessor()

    	rospy.spin()

    def val_map(self, value, istart, istop, ostart, ostop):
        value[value < istart] = istart
        value[value > istop] = istop

        return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))

    def callback(self, data):
    	cv_image = self.bridge.imgmsg_to_cv2(data, "mono16")
    	
        with open("./config/config.json") as f:
            self.config = json.load(f)
            br = self.config["bitRange"]

        cv_image = cv_image.astype(np.float32)

        out = self.val_map(cv_image, br["low"], br["high"], 0, 255)
        out = out.astype(np.uint8)

    	out_image = self.bridge.cv2_to_imgmsg(out, "mono8")
    	self.image_pub.publish(out_image)

if __name__ == "__main__":
    node = Node()

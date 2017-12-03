#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from processor import FrameProcessor, Timer
from geometry_msgs.msg import PoseArray

class Node(object):
    def __init__(self):
        rospy.init_node("bg_subtract", anonymous=True)

        self.image_pub = rospy.Publisher("background_subtracted", Image, queue_size=1000)
        self.debug_pub = rospy.Publisher("rectified", Image, queue_size=1000)
        self.pose_pub = rospy.Publisher("poses", PoseArray, queue_size=1000)

        self.image_sub = rospy.Subscriber("image_raw", Image, self.callback)

        self.bridge = CvBridge()

        inputs = dict()
        for param in ["config", "transform", "mask"]:
            inputs[param] = rospy.get_param("~"+param)

        inputs["bridge"] = self.bridge
        inputs["debug"] = self.debug_pub
        inputs["pose_pub"] = self.pose_pub


        self.fp = FrameProcessor(inputs)

    	rospy.spin()

    def callback(self, data):
    	cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
        # with Timer("core"):
    	subtracted = self.fp.process(cv_image)
        if subtracted is None:
            return
    	out_image = self.bridge.cv2_to_imgmsg(subtracted, "rgb8")
    	self.image_pub.publish(out_image)

if __name__ == "__main__":
    node = Node()

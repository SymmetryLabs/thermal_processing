#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from processor import FrameProcessor

class Node(object):
    def __init__(self):
        rospy.init_node("bg_subtract", anonymous=True)

        self.image_pub = rospy.Publisher("/thermal/background_subtracted", Image, queue_size=1000)
        self.image_sub = rospy.Subscriber("/thermal/image_raw", Image, self.callback)
        self.bridge = CvBridge()

        self.fp = FrameProcessor()

	rospy.spin()

    def callback(self, data):
    	cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
    	subtracted = self.fp.process(cv_image)
    	out_image = self.bridge.cv2_to_imgmsg(cv_image, "rgb8")
    	self.image_pub.publish(out_image)

if __name__ == "__main__":
    node = Node()

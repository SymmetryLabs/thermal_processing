#!/usr/bin/env python
import rospy
import OSC
import yaml
import json
from geometry_msgs.msg import PoseArray

class Listener(object):
    def __init__(self):
        rospy.init_node("osc", anonymous=True)

        ip = rospy.get_param("~ip", "localhost")
        port = int(rospy.get_param("~port", "3131"))
        rospy.loginfo("Connecting to %s on port %s", ip, port)
        self.c = OSC.OSCClient()
        self.c.connect((ip, port))

        rospy.Subscriber("/poses", PoseArray, self.callback)

        rospy.spin()


    def callback(self, data):
        # oh god it hurts
        # this will be more thoughtfully set up once we figure out what data we need to send
        jsonified = json.dumps(yaml.load(data.__str__()))
        oscmsg = OSC.OSCMessage()
        oscmsg.setAddress("/poses")
        oscmsg.append(jsonified)
        self.c.send(oscmsg)

if __name__ == "__main__":
    l = Listener()
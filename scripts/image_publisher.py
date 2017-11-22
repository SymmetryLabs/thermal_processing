#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image

class Publisher(object):
    def __init__(self):
        rospy.init_node("osc", anonymous=True)

        self.width = int(rospy.get_param("~width", "320"))
        self.height = int(rospy.get_param("~height", "256"))
        self.bytes_per_pixel = int(rospy.get_param("~bytes_per_pixel", "2"))
        self.fps = float(rospy.get_param("~fps", "30"))
        self.filename = rospy.get_param("~file")

        self.rate = rospy.Rate(self.fps)
        self.pub = rospy.Publisher("/thermal/image_raw", Image, queue_size=1000)

        self.bytes_per_image = self.width * self.height * self.bytes_per_pixel
        self.encoding = "mono8" if self.bytes_per_pixel == 1 else "mono16"

        # self.data = self.generate_fake_data()
        self.data = self.read_data_from_file()
        self.n_images = self.data.shape[0] / self.bytes_per_image

        i = 0
        while not rospy.is_shutdown() and i < self.n_images:
            img_data = self.data[i:i+self.bytes_per_image]
            img = Image(
                width=self.width,
                height=self.height,
                encoding=self.encoding,
                step=self.width * self.bytes_per_pixel,
                data=img_data.tolist(),
            )
            img.header.stamp = rospy.get_rostime()
            self.pub.publish(img)
            i += 1
            self.rate.sleep()

          
    def read_data_from_file(self):
        return np.fromfile(self.filename, dyype=np.uint8, sep="")

    def generate_fake_data(self):
        n = 10000
        data = np.random.randint(low=0, high=256, size=(n * self.bytes_per_image), dtype=np.uint8)
        return data


        return None

if __name__ == "__main__":
    pub = Publisher()




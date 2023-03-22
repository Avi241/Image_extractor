#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

class ImageTimestampNode:
    def __init__(self):
        self.image1_timestamp = None
        self.image2_timestamp = None
        self.sub1 = rospy.Subscriber("/image_raw", Image, self.image1_callback)
        self.sub2 = rospy.Subscriber("/camera/color/image_raw", Image, self.image2_callback)

    def image1_callback(self, msg):
        self.image1_timestamp = msg.header.stamp
        self.print_timestamps()

    def image2_callback(self, msg):
        self.image2_timestamp = msg.header.stamp
        self.print_timestamps()

    def print_timestamps(self):
        if self.image1_timestamp is not None and self.image2_timestamp is not None:
            rospy.loginfo("Image 1 timestamp: %s, Image 2 timestamp: %s", self.image1_timestamp, self.image2_timestamp)

if __name__ == "__main__":
    rospy.init_node("image_timestamp_node")
    node = ImageTimestampNode()
    rospy.spin()


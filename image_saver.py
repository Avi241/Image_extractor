#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaverNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.folder_path = rospy.get_param("~folder_path", os.path.expanduser("~"))
        if not os.path.isdir(self.folder_path):
            rospy.logerr("Invalid folder path: %s", self.folder_path)
            rospy.signal_shutdown("Invalid folder path")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr("Failed to convert image: %s", e)
            return

        try:
            filename = os.path.join(self.folder_path, "image_%s.jpg" % rospy.get_time())
            cv2.imwrite(filename, cv_image)
            rospy.loginfo("Image saved: %s", filename)
        except Exception as e:
            rospy.logerr("Failed to save image: %s", e)

if __name__ == "__main__":
    rospy.init_node("image_saver_node")
    node = ImageSaverNode()
    rospy.spin()


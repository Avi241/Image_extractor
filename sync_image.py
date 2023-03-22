#!/usr/bin/env python

import os
import cv2
import rospy
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge

def callback(image1, image2, image3):
    # Convert the ROS images to OpenCV format
    bridge = CvBridge()
    cv_image1 = bridge.imgmsg_to_cv2(image1, desired_encoding="bgr8")
    cv_image2 = bridge.imgmsg_to_cv2(image2, desired_encoding="bgr8")
    cv_image3 = bridge.imgmsg_to_cv2(image3, desired_encoding="passthrough")
    cv_image4 = bridge.imgmsg_to_cv2(image3, desired_encoding="16UC1")
    cv_image3 = 255-cv_image3
    cv2.imshow("img",cv_image3)
    cv2.imshow("img2",cv_image4)
    cv2.waitKey(1)

    # Save the images to separate folders
    folder1 = "./thermal_sync"
    folder2 = "./rgb_sync"
    folder3 = "./depth_sync"

    if not os.path.exists(folder1):
        os.makedirs(folder1)
    if not os.path.exists(folder2):
        os.makedirs(folder2)
    if not os.path.exists(folder3):
        os.makedirs(folder3)

    timestamp = rospy.Time.now().to_nsec()
    filename1 = os.path.join(folder1, "image1_%d.jpg" % timestamp)
    filename2 = os.path.join(folder2, "image2_%d.jpg" % timestamp)
    filename3 = os.path.join(folder3, "image3_%d.jpg" % timestamp)
    # cv2.imwrite(filename1, cv_image1)
    # cv2.imwrite(filename2, cv_image2)
    # cv2.imwrite(filename3, cv_image3)
    
def image_subscriber():
    # Subscribe to the two image topics
    sub_image1 = Subscriber('/image_raw', Image)
    sub_image2 = Subscriber('/camera/color/image_raw', Image)
    sub_image3 = Subscriber("/camera/aligned_depth_to_color/image_raw",Image)

    # Synchronize the two image topics
    sync = ApproximateTimeSynchronizer([sub_image1, sub_image2, sub_image3], queue_size=10, slop=0.1)
    sync.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('image_synchronizer')
    image_subscriber()

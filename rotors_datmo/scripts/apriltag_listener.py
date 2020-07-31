#!/usr/bin/env python
import numpy as np
import apriltag
import rospy
import roslib
import cv2
import message_filters
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image

VERBOSE = True

def callback(data):
    # print(data)
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    detector = apriltag.Detector()
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data, "mono8")
    # image = cv2.imread(data, cv2.IMREAD_GRAYSCALE)
    # print(image)
    detections = detector.detect(image)
    print(detections[0])

def detect():
    detector = apriltag.Detector()

    imagepath = "/home/ziyou/Desktop/test1.png"
    image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)
    detections = detector.detect(image)
    print(detections)

    imagepath = "/home/ziyou/Desktop/test2.png"
    image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)
    detections = detector.detect(image)
    print(detections)

    imagepath = "/home/ziyou/Desktop/test3.png"
    image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)
    detections = detector.detect(image)
    print(detections)


def detector():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('tag_detector', anonymous=True)

    rospy.Subscriber("/depth_camera/image_raw", Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    detector()

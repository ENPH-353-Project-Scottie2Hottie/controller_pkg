#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import timeit

# License Plate
# uh = 130
# us = 255
# uv = 255
# lh = 116
# ls = 50
# lv = 86

# Red Line
uh = 10
us = 255
uv = 255
lh = 0
ls = 200
lv = 0

lower_hsv = np.array([lh, ls, lv])
upper_hsv = np.array([uh, us, uv])


def callback(data):
    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
      hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    except CvBridgeError as e:
      print(e)

    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    cv2.imshow("Thresh", mask)
    cv2.waitKey(3)

if __name__ == '__main__':
    bridge = CvBridge()
    rospy.init_node('threshold_test')
    rospy.Subscriber('/R1/pi_camera/image_raw', Image, callback)
    rospy.spin()
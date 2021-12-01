#!/usr/bin/env python

import time
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Red Line
uh = 10
us = 255
uv = 255
lh = 0
ls = 200
lv = 0

lower_hsv = np.array([lh, ls, lv])
upper_hsv = np.array([uh, us, uv])
MIN_RED_PIXELS = 35000

def callback(data):
  try:
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
  except CvBridgeError as e:
    print(e)

  mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
  num_red_pixels = cv2.countNonZero(mask)
  if num_red_pixels > MIN_RED_PIXELS:
    pub.publish("True")
  else:
    pub.publish("False")

if __name__ == '__main__':
    bridge = CvBridge()
    rospy.init_node('red_line_detector')
    pub = rospy.Publisher('/red_line', String, queue_size=1)
    rospy.Subscriber('/R1/pi_camera/image_raw', Image, callback)
    time.sleep(1)
    rospy.spin()
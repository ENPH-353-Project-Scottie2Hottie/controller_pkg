#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time

# Red Line
uh = 10
us = 255
uv = 255
lh = 0
ls = 200
lv = 0

lower_hsv = np.array([lh, ls, lv])
upper_hsv = np.array([uh, us, uv])
MIN_RED_PIXELS = 50000

last_stop_time = 0
TIME_DELAY = 3

def callback(data):
  global last_stop_time
  if time.time() < last_stop_time + TIME_DELAY:
    # pub.publish("False")
    return
  
  try:
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
  except CvBridgeError as e:
    print(e)

  mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
  num_red_pixels = cv2.countNonZero(mask)
  if num_red_pixels > MIN_RED_PIXELS:
    last_stop_time = time.time()
    pub_ped.publish("True")
  else:
    pub_pid.publish("Continue")
    pub_ped.publish("False")


if __name__ == '__main__':
    bridge = CvBridge()
    rospy.init_node('red_line_detector')
    pub_ped = rospy.Publisher('/red_line', String, queue_size=1)
    pub_pid = rospy.Publisher('/path_follow', String, queue_size=1)
    rospy.Subscriber('/R1/pi_camera/image_raw', Image, callback)
    rospy.spin()
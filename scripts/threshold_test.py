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
# uh = 10
# us = 255
# uv = 255
# lh = 0
# ls = 200
# lv = 0

# Blue Cars
# uh = 125
# us = 255
# uv = 210
# lh = 115
# ls = 125
# lv = 100

# License Plate
uh = 0
us = 0
uv = 205
# uv = 140
lh = 0
ls = 0
lv = 90

lower_hsv = np.array([lh, ls, lv])
upper_hsv = np.array([uh, us, uv])

MIN_FRACTION_WHITE = 0.32
MAX_FRACTION_WHITE = 0.455
EROSION_KERNEL_SIZE = 12
DILATION_KERNEL_SIZE = 20
MIN_RECT_AREA = 33000
MAX_RECT_AREA = 45000

def callback(data):
    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
      hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    except CvBridgeError as e:
      print(e)

    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

    erosion_kernel = np.ones((EROSION_KERNEL_SIZE, EROSION_KERNEL_SIZE), np.uint8)
    dilation_kernel = np.ones((DILATION_KERNEL_SIZE, DILATION_KERNEL_SIZE), np.uint8)
    mask = cv2.erode(mask, erosion_kernel)
    mask = cv2.dilate(mask, dilation_kernel)

    rows, cols = mask.shape
    bl_mask = mask[rows/2:4*rows/5, 0:cols/4]
    x, y, w, h = cv2.boundingRect(bl_mask)
    top_y = rows/2 + y
    bot_y = top_y + h
    left_x = x
    right_x = left_x + w

    rect_area = w * h
    # print(rect_area)

    bl_rows, bl_cols = bl_mask.shape
    num_pixels = bl_rows * bl_cols
    num_white = cv2.countNonZero(bl_mask)
    frac_white = num_white * 1.0 / num_pixels
    # print(frac_white)
    
    if rect_area >= MIN_RECT_AREA and rect_area <= MAX_RECT_AREA and frac_white >= MIN_FRACTION_WHITE and frac_white <= MAX_FRACTION_WHITE:
      print("True")
      print(rect_area)
      print(frac_white)

      plate = cv_image[top_y:bot_y, left_x:right_x]
      cv2.imshow("Plate", plate)
      cv2.waitKey(3)

    cv2.imshow("Thresh", mask)
    cv2.waitKey(3)

if __name__ == '__main__':
    bridge = CvBridge()
    rospy.init_node('threshold_test')
    rospy.Subscriber('/R1/pi_camera/image_raw', Image, callback)
    rospy.spin()
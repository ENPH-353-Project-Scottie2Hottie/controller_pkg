#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import timeit
import time

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
lh = 0
ls = 0
lv = 90

lower_hsv = np.array([lh, ls, lv])
upper_hsv = np.array([uh, us, uv])

# MIN_FRACTION_WHITE = 0.32
# MAX_FRACTION_WHITE = 0.455
# EROSION_KERNEL_SIZE = 12
# DILATION_KERNEL_SIZE = 20
# MIN_RECT_AREA = 33000
# MAX_RECT_AREA = 45000

MIN_FRACTION_WHITE = 0.38
MAX_FRACTION_WHITE = 0.42
MIN_RECT_AREA = 24000
MAX_RECT_AREA = 27000


def callback(data):
    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
      hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    except CvBridgeError as e:
      print(e)

    hsv = cv2.GaussianBlur(hsv,(5,5),cv2.BORDER_DEFAULT)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

    # erosion_kernel = np.ones((EROSION_KERNEL_SIZE, EROSION_KERNEL_SIZE), np.uint8)
    # dilation_kernel = np.ones((DILATION_KERNEL_SIZE, DILATION_KERNEL_SIZE), np.uint8)
    # mask = cv2.erode(mask, erosion_kernel)
    # mask = cv2.dilate(mask, dilation_kernel)

    se1 = cv2.getStructuringElement(cv2.MORPH_RECT, (20,20))
    se2 = cv2.getStructuringElement(cv2.MORPH_RECT, (35,35))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, se1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, se2)

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
    
    # if rect_area >= MIN_RECT_AREA and rect_area <= MAX_RECT_AREA and frac_white >= MIN_FRACTION_WHITE and frac_white <= MAX_FRACTION_WHITE:
    if rect_area > 0 and frac_white > 0:
      print("True")
      print(rect_area)
      print(frac_white)

      plate = cv_image[top_y:bot_y, left_x:right_x]
      cv2.imshow("Plate", plate)
      cv2.waitKey(3)
      # filename = str(time.time()) + '.jpg'
      # cv2.imwrite(filename, plate)

    cv2.imshow("Thresh", mask)
    cv2.waitKey(3)

if __name__ == '__main__':
    bridge = CvBridge()
    rospy.init_node('license_plate_detector')
    rospy.Subscriber('/R1/pi_camera/image_raw', Image, callback)
    rospy.spin()
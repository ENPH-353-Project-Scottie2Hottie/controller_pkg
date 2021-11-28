#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time

# Blue Cars
uh = 125
us = 255
uv = 210
lh = 115
ls = 125
lv = 100
blue_lower_hsv = np.array([lh, ls, lv])
blue_upper_hsv = np.array([uh, us, uv])

# License Plate Big
uh = 0
us = 0
uv = 205
lh = 0
ls = 0
lv = 90
white_lower_hsv = np.array([lh, ls, lv])
white_upper_hsv = np.array([uh, us, uv])

# Rev Plate
uh = 0
us = 0
uv = 255
lh = 0
ls = 0
lv = 0
plate_lower_hsv = np.array([lh, ls, lv])
plate_upper_hsv = np.array([uh, us, uv])

best_sample = None
best_masked_sample = None
best_ratio = 0
first_sample_time = 0

def callback(data):
    global best_sample
    global first_sample_time
    global best_masked_sample
    global best_ratio

    if best_ratio != 0 and time.time() > first_sample_time + 2:
      height, width = best_masked_sample.shape
      cv2.imshow("Best", best_sample)
      cv2.waitKey(3)

      best_masked_sample = morph(best_masked_sample, 3, 5)
      _, contours, _ = cv2.findContours(best_masked_sample, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      max_contour = max(contours, key=len)
      rect = cv2.minAreaRect(max_contour)
      box = cv2.boxPoints(rect)
      box = np.int0(box)
      tl = box[2]
      tr = box[3]
      bl = box[1]
      br = box[0]

      widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
      widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
      maxWidth = max(int(widthA), int(widthB))

      heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
      heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
      maxHeight = max(int(heightA), int(heightB))

      if maxWidth < maxHeight:
        temp = bl
        bl = br
        br = tr
        tr = tl
        tl = temp
        temp = maxWidth
        maxWidth = maxHeight
        maxHeight = temp

      pts1 = np.float32([tl, tr, br, bl])
      pts2 = np.float32([[0, 0], [maxWidth-1, 0], [maxWidth-1, maxHeight-1], [0, maxHeight-1]])
      matrix = cv2.getPerspectiveTransform(pts1, pts2)
      result = cv2.warpPerspective(best_sample, matrix, (maxWidth, maxHeight))
      cv2.imshow('Lic. Plate', result)
      cv2.waitKey(3)

      pos_img = best_sample[0:min(tl[1], tr[1])-1, 0:width-1]
      cv2.imshow('Position', pos_img)
      cv2.waitKey(3)

      best_sample = None
      best_ratio = 0

    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
      hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    except CvBridgeError as e:
      print(e)

    hsv = cv2.GaussianBlur(hsv,(5,5),cv2.BORDER_DEFAULT)

    plate_mask = cv2.inRange(hsv, plate_lower_hsv, plate_upper_hsv)
    plate_mask = cv2.bitwise_not(plate_mask)

    blue_mask = cv2.inRange(hsv, blue_lower_hsv, blue_upper_hsv)
    blue_mask = morph(blue_mask, 3, 5)

    x, y, w, h = cv2.boundingRect(blue_mask)
    blue_top_y = y
    blue_bot_y = blue_top_y + h
    blue_left_x = x
    blue_right_x = blue_left_x + w

    blue_area = w * h

    if blue_area > 0:
      blue_car = hsv[blue_top_y:blue_bot_y, blue_left_x:blue_right_x]

      white_mask = cv2.inRange(blue_car, white_lower_hsv, white_upper_hsv)
      white_mask = morph(white_mask, 12, 20)

      x, y, w, h = cv2.boundingRect(white_mask)
      white_top_y = blue_top_y + y
      white_bot_y = white_top_y + h
      white_left_x = blue_left_x + x
      white_right_x = white_left_x + w

      white_area = w * h

      if white_area > 0:
        ratio = 1.0 * w / h
        
        if ratio < 0.82 and ratio > 0.7 and white_area > 20000:
          # print("ratio=", ratio)
          # print("area=", white_area)
          plate = cv_image[white_top_y:white_bot_y, white_left_x:white_right_x]
          cv2.imshow("Plate", plate)
          cv2.waitKey(3)

          binary_plate = plate_mask[white_top_y:white_bot_y, white_left_x:white_right_x]

          if best_ratio == 0:
            best_sample = plate
            best_ratio = ratio
            best_masked_sample = binary_plate
            first_sample_time = time.time()
          else:
            if ratio > best_ratio:
              best_ratio = ratio
              best_sample = plate
              best_masked_sample = binary_plate


    # cv2.imshow("Thresh", mask)
    # cv2.waitKey(3)

def morph(mask, close_size, open_size):
  if close_size:
    se1 = cv2.getStructuringElement(cv2.MORPH_RECT, (close_size,close_size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, se1)
  if open_size:
    se2 = cv2.getStructuringElement(cv2.MORPH_RECT, (open_size,open_size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, se2)
  return mask

if __name__ == '__main__':
    bridge = CvBridge()
    rospy.init_node('threshold_test')
    rospy.Subscriber('/R1/pi_camera/image_raw', Image, callback)
    rospy.spin()
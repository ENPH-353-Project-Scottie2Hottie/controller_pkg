#!/usr/bin/env python

import rospy
import numpy as np
import string
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import time
import tensorflow as tf
from tensorflow.keras import models
from tensorflow.python.keras.backend import set_session

sess1 = tf.Session()
graph1 = tf.get_default_graph()
set_session(sess1)

model_path = '/home/fizzer/ros_ws/src/controller_pkg/node/saved_model'
plate_model = models.load_model(model_path)
alphabet = string.ascii_uppercase + '0123456789'

# Blue
blue_lower_hsv = np.array([115, 125, 100])
blue_upper_hsv = np.array([125, 255, 210])

# White
white_lower_hsv = np.array([0, 0, 90])
white_upper_hsv = np.array([0, 0, 205])

# Rev Plate
plate_lower_hsv = np.array([0, 0, 0])
plate_upper_hsv = np.array([0, 0, 255])

best_sample = None
best_masked_sample = None
best_ratio = 0
first_sample_time = 0
SAMPLE_INTERVAL = 2
MAX_PLATE_HEIGHT = 40
# sample_count = 0
# set_num = 0
PLATES_TO_CHECK = 6
checked_positions = np.zeros(PLATES_TO_CHECK)
finished = False
sent_end_message = False
running = False

team_id = 'Scottie'
password = '2Hottie'
start_msg = team_id + ',' + password + ',0,SB04'
end_msg = team_id + ',' + password + ',-1,SB04'

def callback(data):
  global best_sample
  global first_sample_time
  global best_masked_sample
  global best_ratio
  global sample_count
  global sess1
  global graph1
  global checked_positions
  global finished
  global sent_end_message
  global running

  if running:
    pub_status.publish('Started')

  if not sent_end_message:
    if best_ratio != 0 and rospy.get_time() > first_sample_time + SAMPLE_INTERVAL:
      if finished and not sent_end_message:
        rospy.sleep(0.2)
        pub_status.publish('Done')
        pub.publish(end_msg)
        sent_end_message = True
        running = False
        return
      # cv2.imshow("Best", best_sample)
      # cv2.waitKey(3)
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

      if maxWidth <= maxHeight:
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
      license_plate = cv2.warpPerspective(best_sample, matrix, (maxWidth, maxHeight))
      plate_height, plate_width, _ = license_plate.shape
      height_diff = plate_height - MAX_PLATE_HEIGHT
      if height_diff > 0:
        license_plate = license_plate[height_diff/2:height_diff/2+MAX_PLATE_HEIGHT, 0:plate_width]
      # cv2.imshow('Lic. Plate', license_plate)
      # cv2.waitKey(3)

      top_plate = min(tl[1], tr[1])
      pos_img = best_sample[top_plate/2:top_plate, :]
      # cv2.imshow('Position', pos_img)
      # cv2.waitKey(3)


      img = license_plate
      _, width, _ = img.shape
      resize_dim = (42,40)

      let1 = img[:, :int(0.25*width)]
      let1 = cv2.resize(let1, resize_dim, interpolation = cv2.INTER_AREA)
      let1 = cv2.cvtColor(let1, cv2.COLOR_BGR2GRAY)
      let1 = cv2.adaptiveThreshold(let1,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                cv2.THRESH_BINARY_INV,21,10)
      
      let2 = img[:, int(0.19*width):int(0.44*width)]
      let2 = cv2.resize(let2, resize_dim, interpolation = cv2.INTER_AREA)
      let2 = cv2.cvtColor(let2, cv2.COLOR_BGR2GRAY)
      let2 = cv2.adaptiveThreshold(let2,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                cv2.THRESH_BINARY_INV,21,10)

      num3 = img[:, int(0.53*width):int(0.78*width)]
      num3 = cv2.resize(num3, resize_dim, interpolation = cv2.INTER_AREA)
      num3 = cv2.cvtColor(num3, cv2.COLOR_BGR2GRAY)
      num3 = cv2.adaptiveThreshold(num3,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                cv2.THRESH_BINARY_INV,21,10)
      
      num4 = img[:, int(0.7*width):int(0.95*width)]
      num4 = cv2.resize(num4, resize_dim, interpolation = cv2.INTER_AREA)
      num4 = cv2.cvtColor(num4, cv2.COLOR_BGR2GRAY)
      num4 = cv2.adaptiveThreshold(num4,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                cv2.THRESH_BINARY_INV,21,10)

      pos_num = pos_img[:, int(0.5*width):int(1*width)]
      pos_num = cv2.resize(pos_num, resize_dim, interpolation = cv2.INTER_AREA)
      pos_num = cv2.cvtColor(pos_num, cv2.COLOR_BGR2GRAY)
      pos_num = cv2.adaptiveThreshold(pos_num,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                cv2.THRESH_BINARY_INV,21,10)

      let1 = let1/255.
      let2 = let2/255.
      num3 = num3/255.
      num4 = num4/255.
      pos_num = pos_num/255.

      let1 = let1.reshape(1,40,42,1)
      let2 = let2.reshape(1,40,42,1)
      num3 = num3.reshape(1,40,42,1)
      num4 = num4.reshape(1,40,42,1)
      pos_num = pos_num.reshape(1,40,42,1)
      
      with graph1.as_default():
        set_session(sess1)

        let1_pred = alphabet[np.argmax(plate_model.predict(let1)[0][:26])]
        let2_pred = alphabet[np.argmax(plate_model.predict(let2)[0][:26])]
        num3_pred = alphabet[np.argmax(plate_model.predict(num3)[0][26:36]) + 26]
        num4_pred = alphabet[np.argmax(plate_model.predict(num4)[0][26:36]) + 26]
        pos_pred = np.argmax(plate_model.predict(pos_num)[0])
        if pos_pred >= 26:
          pos_pred = alphabet[pos_pred]
          prediction = str(pos_pred) + ',' + let1_pred + let2_pred + str(num3_pred) + str(num4_pred)
          pub.publish(team_id + ',' + password + ',' + prediction)
          checked_positions[int(pos_pred) - 1] = 1
          if (checked_positions == np.ones(PLATES_TO_CHECK)).all():
            finished = True

      # cv2.imwrite('plate' + str(sample_count + 6*set_num) + '.png', license_plate)
      # cv2.imwrite('pos' + str(sample_count + 6*set_num) + '.png', pos_img)
      # sample_count += 1

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
        
        if ratio < 0.82 and ratio > 0.67 and white_area > 20000:
          # print("ratio=", ratio)
          # print("area=", white_area)
          plate = cv_image[white_top_y:white_bot_y, white_left_x:white_right_x]
          # cv2.imshow("Plate", plate)
          # cv2.waitKey(3)

          binary_plate = plate_mask[white_top_y:white_bot_y, white_left_x:white_right_x]

          if best_ratio == 0:
            best_sample = plate
            best_ratio = ratio
            best_masked_sample = binary_plate
            first_sample_time = rospy.get_time()
          else:
            if ratio > best_ratio:
              best_ratio = ratio
              best_sample = plate
              best_masked_sample = binary_plate

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
  pub = rospy.Publisher('/license_plate', String, queue_size=1)
  pub_status = rospy.Publisher('/lpd_status', String, queue_size=1)
  rospy.Subscriber('/R1/pi_camera/image_raw', Image, callback)
  time.sleep(2)
  pub.publish(start_msg)
  pub_status.publish('Started')
  running = True
  rospy.spin()
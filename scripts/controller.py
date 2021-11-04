#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError
import timeit

#Constants
linear_p = 1.5
angular_p = 2

def callback(data):
    move = Twist()
    threshold = 120
    try:
      cv_image = bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
      print(e)

    (rows,cols) = cv_image.shape
    _, thresh = cv2.threshold(cv_image[rows - 300 - 1:,:], threshold, 255, cv2.THRESH_BINARY_INV)

    # cv2.imshow("View", thresh)
    # cv2.waitKey(3)

    (new_rows, new_cols) = thresh.shape
    mid_x = new_cols // 2
    y_threshold = new_rows

    centre_M = cv2.moments(thresh)

    if centre_M["m00"] != 0:
        x_centre = int(centre_M["m10"] / centre_M["m00"])
        y_centre = int(centre_M["m01"] / centre_M["m00"])
    else:
        x_centre = mid_x
        y_centre = 0

    # cv2.circle(thresh, (x_centre, y_centre), 10, 255, -1)
    # num_path_pixels = 0
    # for y in range(top_y, rows):
    #     for x in range(0, cols):
    #         if cv_image[y, x] < threshold:
    #             x_centre += x
    #             y_centre += y
    #             num_path_pixels += 1
    
    # try:
    #     x_centre //= num_path_pixels
    # except ZeroDivisionError:
    #     x_centre = mid_x
    
    # try:
    #     y_centre //= num_path_pixels
    # except ZeroDivisionError:
    #     y_centre = top_y

    if y_centre < y_threshold:
        move.linear.x = float(y_threshold - y_centre) / 300 * linear_p
    else:
        move.linear.x = 0
    
    move.angular.z = 2 * float(mid_x - x_centre) / cols * angular_p

    print('x_centre:', x_centre)
    print('y_centre:', y_centre)
    print('move.linear.x', (y_threshold - y_centre) / 300 * linear_p)
    print('move.angular.z', 2 * (mid_x - x_centre) / cols * angular_p)

    pub.publish(move)

if __name__ == '__main__':
    bridge = CvBridge()
    rospy.init_node('main_controller')
    pub_cmd_vel = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
    pub_license = rospy.Publisher('/license_plate', String, queue_size=1)
    rospy.Subscriber('/R1/camera1/image_raw', Image, callback)
    # rospy.Subscriber('/clock')
    rospy.spin()
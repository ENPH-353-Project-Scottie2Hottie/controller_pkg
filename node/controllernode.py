#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import numpy as np
import matplotlib as plt
import cv2
from cv_bridge import CvBridge, CvBridgeError

velocity = 0.1
multiplier = 4

# img = cv.imread('p1.png',cv.IMREAD_COLOR)
# img = cv.medianBlur(img,5)

# # Convert BGR to HSV
# hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

# uh = 130
# us = 255
# uv = 255
# lh = 116
# ls = 50
# lv = 86
# lower_hsv = np.array([lh,ls,lv])
# upper_hsv = np.array([uh,us,uv])

# # Threshold the HSV image to get only blue colors
# mask = cv.inRange(hsv, lower_hsv, upper_hsv)




def callback(data):
	# get image
	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
	cv_image = cv2.medianBlur(cv_image,5)

	cv2.imwrite("robot_view.png",cv_image)

	# # Convert BGR to HSV
	hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)


	
	# Road following
	uh = 1
	us = 2
	uv = 129
	lh = 0
	ls = 0
	lv = 83

	lower_hsv = np.array([lh,ls,lv])
	upper_hsv = np.array([uh,us,uv])

	# Threshold the HSV image to get only blue colors
	binary = cv2.inRange(hsv, lower_hsv, upper_hsv)

	dimensions = cv_image.shape
	height = dimensions[0]
	width = dimensions[1]
	centreline = width / 2
	


	cropped = binary[int(0.6*height):,:]
	# grayscale = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
	# ret, binary = cv2.threshold(grayscale, 230, 255,cv2.THRESH_BINARY_INV)
	M = cv2.moments(binary)


	cX = int(M["m10"] / M["m00"])
	cY = int(M["m01"] / M["m00"])


	circle_image = cv2.circle(binary, (cX, cY), 4, (0,0,255))
	cv2.imshow("test", circle_image)
	cv2.waitKey(1)


	angular = float(multiplier*(centreline - cX)) / centreline

	# Publisher: publish velocity commands

	move = Twist()
	move.linear.x = 0

	move.angular.z = angular
	move.linear.x = velocity

	# pub.publish(move)
	

# Subscriber: get image data

if __name__ == "__main__":
	bridge = CvBridge()
	rospy.init_node('controller')
	pub = rospy.Publisher('/R1/cmd_vel', Twist, 
  	queue_size=1)
	rospy.Subscriber("/R1/pi_camera/image_raw", Image, callback)
	rospy.spin()

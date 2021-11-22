#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import numpy as np
import matplotlib as plt
import cv2
from cv_bridge import CvBridge, CvBridgeError

velocity = 0.15
multiplier = 0.015
v_multiplier = 0.001


def callback(data):
	# get image
	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
	cv_image = cv2.medianBlur(cv_image,5)

	# cv2.imwrite("robot_view.png",cv_image)

	# Get dimensions of image
	dimensions = cv_image.shape
	height = dimensions[0]
	width = dimensions[1]
	centreline = width / 2

	# Convert BGR to HSV
	hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

	# HSV thresholding for white lines
	uh = 50
	us = 150
	uv = 255
	lh = 0
	ls = 0
	lv = 245


	lower_hsv = np.array([lh,ls,lv])
	upper_hsv = np.array([uh,us,uv])


	binary = cv2.inRange(hsv, lower_hsv, upper_hsv)

	
	# HSV thresholding for red line
	uh = 10
	us = 255
	uv = 255
	lh = 0
	ls = 200
	lv = 0


	lower_hsv = np.array([lh,ls,lv])
	upper_hsv = np.array([uh,us,uv])

	red_line_detector = cv2.inRange(hsv, lower_hsv, upper_hsv)


	###################################################################
	

	# Construct right guide-line equation (w = m*h + b):
	m = 8.7/6
	b = 2.4*width/24.6


	reference = int(0.75 * width)
	

	cropped = binary[int(0.7*height):,int(0.5*width):]

	mask = np.zeros((height,width,1), np.uint8)
	cv2.rectangle(mask, (int(0.5*width),int(0.75*height)),(width,height),(255,0,0),-1)

	masked_img = cv2.bitwise_and(binary, mask)
	cv2.imshow("masked", masked_img)

	M = cv2.moments(masked_img)


	cX = int(M["m10"] / M["m00"])
	cY = int(M["m01"] / M["m00"])


	desiredX = int(m*cY + b)

	currState = cv2.circle(cv_image, (cX, cY), 10, (0,0,255))
	desiredState = cv2.circle(cv_image, (desiredX, cY), 10, (255,0,0))
	cv2.imshow("", desiredState)
	cv2.waitKey(1)

	error = desiredX - cX

	angular = float(multiplier*(error)) 
	print(angular)

	# Publisher: publish velocity commands

	move = Twist()

	move.angular.z = angular
	move.linear.x = velocity*(1 - v_multiplier*abs(error))

	pub.publish(move)
	

# Subscriber: get image data

if __name__ == "__main__":
	bridge = CvBridge()
	rospy.init_node('controller')
	pub = rospy.Publisher('/R1/cmd_vel', Twist, 
  	queue_size=1)
	rospy.Subscriber("/R1/pi_camera/image_raw", Image, callback)
	rospy.spin()

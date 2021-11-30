#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time

velocity = 0.15
a_multiplier = 0.006
v_multiplier = 0.002
flag = 0
mask = None

last_go_time = 0
TIME_DELAY = 1.5
begin = True
start_time = 0
START_DELAY = 1.5

uh = 50
us = 150
uv = 255
lh = 0
ls = 0
lv = 245

lower_hsv = np.array([lh,ls,lv])
upper_hsv = np.array([uh,us,uv])
prev_state = "Go"

red_line = False
pedestrian = True

def pid_callback(data):
	global last_go_time
	global flag
	global mask
	global begin
	global start_time
	global prev_state

	if red_line and pedestrian:
		curr_state = "Stop"
	elif red_line and not pedestrian:
		curr_state = "Go"
	else:
		curr_state = "Follow"

	if begin:
		start_time = rospy.get_time()
		begin = False
	elif rospy.get_time() < start_time + START_DELAY:
		start()
	elif rospy.get_time() < last_go_time + TIME_DELAY:
		return
	elif curr_state == "Stop":
		stop()
		prev_state = curr_state
	elif curr_state == "Go":
		go()
		prev_state = curr_state
		last_go_time = rospy.get_time()
	else:
		if prev_state != "Stop":
			move = Twist()
			cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
			cv_image = cv2.medianBlur(cv_image,5)

			dimensions = cv_image.shape
			height = dimensions[0]
			width = dimensions[1]

			hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
			binary = cv2.inRange(hsv, lower_hsv, upper_hsv)

			b_shift = 50
			m = 8.7/6
			b = 2.4*width/24.6 + b_shift

			if flag == 0:
				mask = np.zeros((height,width,1), np.uint8)
				cv2.rectangle(mask, (int(0.5*width),int(0.75*height)),(width,height),(255,0,0),-1)
				flag = 1

			masked_img = cv2.bitwise_and(binary, mask)

			M = cv2.moments(masked_img)

			if M["m00"] != 0:
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])

				desiredX = int(m*cY + b)

				error = desiredX - cX

				angular = float(a_multiplier*(error)) 

				move.angular.z = angular
				move.linear.x = velocity*(1 - v_multiplier*abs(error))

				pub.publish(move)
	
def red_line_callback(msg):
	global red_line
	if msg.data == "True":
		red_line = True
	else:
		red_line = False

def ped_callback(msg):
	global pedestrian
	if msg.data == "False":
		pedestrian = False
	else:
		pedestrian = True

def stop():
	move = Twist()
	move.linear.x = 0
	move.angular.z = 0
	pub.publish(move)

def go():
	move = Twist()
	move.angular.x = 0
	move.linear.x = velocity*2.5
	pub.publish(move)

def start():
	move = Twist()
	move.linear.x = 0.25
	move.angular.z = 0.4
	pub.publish(move)

if __name__ == "__main__":
	rospy.init_node('pid_controller')
	bridge = CvBridge()
	pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
	rospy.Subscriber("/R1/pi_camera/image_raw", Image, pid_callback)
	rospy.Subscriber("/red_line", String, red_line_callback)
	rospy.Subscriber("/pedestrian", String, ped_callback)
	time.sleep(1)
	rospy.spin()

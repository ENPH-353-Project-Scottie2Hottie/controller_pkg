#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time

prev_frame = None
has_prev_frame = False
MAX_NON_ZERO = 100
TIME_GAP = 0.1
prev_frame_time = 0

def callback(data):
    global prev_frame
    global has_prev_frame
    global prev_frame_time

    if rospy.get_time() < prev_frame_time + TIME_GAP:
        pub.publish("True")
    else:
        prev_frame_time = rospy.get_time()
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
            if not has_prev_frame:
                has_prev_frame = True
                prev_frame = cv_image
            stack_image = cv2.subtract(cv_image, prev_frame)
            gray = cv2.cvtColor(stack_image, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray,25,255,cv2.THRESH_BINARY)
            prev_frame = cv_image
            has_prev_frame = True
            num_non_zero = cv2.countNonZero(thresh)
            if num_non_zero < MAX_NON_ZERO:
                pub.publish("False")
            else:
                pub.publish("True")

        except CvBridgeError as e:
            prev_frame = None
            has_prev_frame = False
            print(e)

if __name__ == '__main__':
    bridge = CvBridge()
    rospy.init_node('pedestrian_detector')
    rospy.Subscriber('/R1/pi_camera/image_raw', Image, callback)
    pub = rospy.Publisher('/pedestrian', String, queue_size=1)
    time.sleep(1)
    rospy.spin()
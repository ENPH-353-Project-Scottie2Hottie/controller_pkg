#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

prev_frame = None
has_prev_frame = False
moving = False
MAX_NON_ZERO = 100

def callback(data):
    global prev_frame
    global has_prev_frame
    global prev_frame_time
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        if moving:
            prev_frame = None
            has_prev_frame = False
            rows, cols, channels = cv_image.shape
            blank_image = np.zeros((rows,cols), np.uint8)
            cv2.imshow("Background Sub.", blank_image)
            cv2.waitKey(3)
        elif not has_prev_frame:
            prev_frame = cv_image
            has_prev_frame = True
            rows, cols, channels = cv_image.shape
            blank_image = np.zeros((rows,cols), np.uint8)
            cv2.imshow("Background Sub.", blank_image)
            cv2.waitKey(3)
        else:
            stack_image = cv2.subtract(cv_image, prev_frame)
            gray = cv2.cvtColor(stack_image, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray,25,255,cv2.THRESH_BINARY)
            cv2.imshow("Background Sub.", thresh)
            cv2.waitKey(3)
            prev_frame = cv_image
            has_prev_frame = True
            num_non_zero = cv2.countNonZero(thresh)
            if num_non_zero > MAX_NON_ZERO:
                print("STOP")
            else:
                print("GO")

    except CvBridgeError as e:
        prev_frame = None
        has_prev_frame = False
        print(e)

if __name__ == '__main__':
    bridge = CvBridge()
    rospy.init_node('bkg_sub_test')
    rospy.Subscriber('/R1/pi_camera/image_raw', Image, callback)
    rospy.spin()
#!/usr/bin/env python

import numpy as np
import cv2 as cv
import time

img1 = cv.imread('frame1.png',cv.IMREAD_COLOR)
img2 = cv.imread('frame2.png',cv.IMREAD_COLOR)

stack_image = cv.subtract(img1, img2)
gray = cv.cvtColor(stack_image, cv.COLOR_BGR2GRAY)
_, thresh = cv.threshold(gray,15,255,cv.THRESH_BINARY)

window_name = "Background Subtraction"
cv.namedWindow(window_name)
while(1):
    cv.imshow(window_name,thresh)
    k = cv.waitKey(1) & 0xFF
    if k == 27:
        break

    time.sleep(0.1)
    
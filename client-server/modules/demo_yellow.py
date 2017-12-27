#!/usr/bin/env python

import numpy as np
import cv2
from PIL import Image
import time

def get_frame(frame, DEBUG=False):
    lower,upper=([0, 75, 75], [75, 255, 255])
    lower = np.array(lower, dtype = "uint8")
    upper = np.array(upper, dtype = "uint8")
    mask = cv2.inRange(frame, lower, upper)
    output = cv2.bitwise_and(frame, frame, mask = mask)
    ret,thresh = cv2.threshold(output,10,255,1)

    if DEBUG:
        cv2.imshow("yellow", np.hstack([thresh, output]))
    return thresh
img = cv2.imread('/home/kunal/Desktop/img2.png')

cv2.imshow('hey', img)

get_frame(img,True)
if cv2.waitKey(10000) & 0xFF == ord('q'):       #q to quit
    cv2.destroyAllWindows()

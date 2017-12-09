#!/usr/bin/env python

from PIL import Image
import cv2
import get_yellow
import numpy as np

def main():
    # Select and start Camera.
    DEVICE_NO = 0
    capture = cv2.VideoCapture(DEVICE_NO)
    if not capture:
        print "Failed to open camera number %d" %(DEVICE_NO)
        return False
    while True:
        if cv2.waitKey(100) & 0xFF == ord('q'):       #q to quit
            cv2.destroyAllWindows()
            break

        ret, frame = capture.read()
        if ret==False:
            print "Error !"
            return False
        cv2.imshow('Video Capture', frame)
        line = get_yellow.get_frame(frame, DEBUG=True)
#        cv2.imshow("yellow", get_yellow.get_frame(frame))
    return True

if __name__ == "__main__":
    print main()

#!/usr/bin/env python

from PIL import Image
import cv2
import get_yellow
import numpy as np
import time

from imutils.video.pivideostream import PiVideoStream


resolution = (1920, 1080)

def main():
    # Select and start Camera.
    vs = PiVideoStream(resolution).start()
    time.sleep(2)
    while True:
        if cv2.waitKey(100) & 0xFF == ord('q'):       #q to quit
            cv2.destroyAllWindows()
            break

        frame = vs.read()
        if frame is None:
            print "Error !"
            return False
        cv2.imshow('Video Capture', frame)
        line = get_yellow.get_frame(frame, DEBUG=True)
#        cv2.imshow("yellow", get_yellow.get_frame(frame))
    return True

if __name__ == "__main__":
    print main()

#!/usr/bin/env python

from PIL import Image
import cv2
import get_yellow
import numpy as np
import time
import detectrectangles

from imutils.video.pivideostream import PiVideoStream
from pi_camera_settings import camera_settings



def main():
    # Select and start Camera.
    vs = PiVideoStream(camera_settings['resolution'],camera_settings['fps']).start()
    time.sleep(2)
    cnt = 0
    strt = time.time()
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):       #q to quit
            cv2.destroyAllWindows()
            break

        frame = cv2.flip(vs.read(),camera_settings['flip'])
        cnt+=1
        if frame is None:
            print "Error !"
            return False
        cv2.imshow('Video Capture', frame)
        line = get_yellow.get_frame(frame, DEBUG=True)
#        cv2.imshow("yellow", get_yellow.get_frame(frame))
        cv2.imshow('Canny', cv2.Canny(line,100,200))
        # rectangle = detectrectangles.Rectangle(100, line)
        # rectangle.has_rectangle(img=frame, DEBUG=False)
    print "[*]FPS =",cnt/(time.time()-strt)
    return True

if __name__ == "__main__":
    print main()

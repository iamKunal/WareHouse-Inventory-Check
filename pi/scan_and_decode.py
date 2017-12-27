#!/usr/bin/env python
import zbar

from PIL import Image
import cv2
import detecttriangle
import hascodes
import numpy as np
import time
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

        frame = vs.read()
        cnt+=1
        if frame is None:
            print "Error !"
            return False
        cv2.imshow('Video Capture', frame)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,127,255,1)
#        cv2.imshow('Gray', np.hstack([gray,thresh]))
        codes = hascodes.GetCodes()
        codes.scan(gray)
        if codes.hasQR:
#            print len(codes.qr)
            for c in codes.qr:
                print c.data
        if codes.hasBar:
#            print len(codes.bar)
            for c in codes.bar:
                print c.data
        print "Frame Done"
        triangle = detecttriangle.DetectTriangle(100, gray)
        triangle_present = triangle.has_triangle(frame, DEBUG=False)
        triangle_location = triangle.location
    print "[*]FPS =",cnt/(time.time()-strt)
    return True

if __name__ == "__main__":
    print main()

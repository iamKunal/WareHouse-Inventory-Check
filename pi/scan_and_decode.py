#!/usr/bin/env python
import zbar

from PIL import Image
import cv2
import detecttriangle
import hascodes
import numpy as np
import time
from imutils.video.pivideostream import PiVideoStream

def main():
    # Select and start Camera.
    vs = PiVideoStream().start()
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

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#        ret,thresh = cv2.threshold(gray,127,255,1)
#        cv2.imshow('Gray', np.hstack([gray,thresh]))
#        codes = hascodes.GetCodes()
#        codes.scan(gray)
##        if codes.hasQR:
###            print len(codes.qr)
##            for c in codes.qr:
##                print c.data
#        if codes.hasBar:
##            print len(codes.bar)
#            for c in codes.bar:
#                print c.data
#        print "Frame Done"
#        triangle = detecttriangle.DetectTriangle(100, gray)
#        triangle_present = triangle.has_triangle(frame)
#        triangle_location = triangle.location
        lower,upper=([0, 146, 190], [142, 255, 255])
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")
        # find the colors within the specified boundaries and apply
        # the mask
        mask = cv2.inRange(frame, lower, upper)
        output = cv2.bitwise_and(frame, frame, mask = mask)
        ret,thresh = cv2.threshold(output,10,255,1)
        # show the images
        cv2.imshow("yellow", np.hstack([thresh, output]))
    return True

if __name__ == "__main__":
    print main()

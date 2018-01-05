#!/usr/bin/env python
import zbar

from PIL import Image
import cv2
import detecttriangle
import hascodes
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
    return True

if __name__ == "__main__":
    print main()

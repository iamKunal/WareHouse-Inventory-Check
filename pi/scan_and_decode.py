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
import imutils

def Zoom(cv2Object, zoomSize):
    # Resizes the image/video frame to the specified amount of "zoomSize".
    # A zoomSize of "2", for example, will double the canvas size
    # cv2Object = imutils.resize(cv2Object, width=(zoomSize * cv2Object.shape[1]))
    # center is simply half of the height & width (y/2,x/2)
    center = (cv2Object.shape[0]/2,cv2Object.shape[1]/2)
    # cropScale represents the top left corner of the cropped frame (y/x)
    # cropScale = (center[0]/zoomSize, center[1]/zoomSize)
    dim = [[center[1]-center[1]/zoomSize, center[0]-center[0]/zoomSize], [center[1]+center[1]/zoomSize, center[0]+center[0]/zoomSize]]


    # The image/video frame is cropped to the center with a size of the original picture
    # image[y1:y2,x1:x2] is used to iterate and grab a portion of an image
    # (y1,x1) is the top left corner and (y2,x1) is the bottom right corner of new cropped frame.
    cv2Object = cv2Object[dim[0][0]:dim[0][1], dim[1][0]:dim[1][1]]
    return cv2Object
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
        triangle = detecttriangle.DetectTriangle(30, gray)
        triangle_present = triangle.has_triangle(frame, DEBUG=False)
        triangle_location = triangle.location
        zoom = Zoom(frame, 2)
        cv2.imshow("zoom", zoom)
    print "[*]FPS =",cnt/(time.time()-strt)
    return True

if __name__ == "__main__":
    print main()

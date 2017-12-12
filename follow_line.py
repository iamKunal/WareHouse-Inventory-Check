#!/usr/bin/env python

import urllib
from PIL import Image
import cv2
import get_yellow
import numpy as np
import detectrectangles

# IP webcam URL
url = 'http://192.168.0.105:4747/cam/1/frame.jpg'
def main():
    # # Select and start Camera.
    # DEVICE_NO = 0
    # capture = cv2.VideoCapture(DEVICE_NO)
    # if not capture:
    #     print "Failed to open camera number %d" %(DEVICE_NO)
    #     return False
    while True:
        imgResp=urllib.urlopen(url)
        imgNp=np.array(bytearray(imgResp.read()),dtype=np.uint8)
        img=cv2.imdecode(imgNp,-1)
        if cv2.waitKey(100) & 0xFF == ord('q'):       #q to quit
            cv2.destroyAllWindows()
            break
        height,width = img.shape[:2]
        frame = img[10:width-10,0:height]
        # ret, frame = capture.read()
        # if ret==False:
        #     print "Error !"
        #     return False
        cv2.imshow('Video Capture', frame)
        line = get_yellow.get_frame(frame, DEBUG=False)
#        cv2.imshow("yellow", get_yellow.get_frame(frame))
        cv2.imshow('Canny', cv2.Canny(line,100,200))
        rectangle = detectrectangles.Rectangle(100, line)
        rectangle.has_rectangle(img=None, DEBUG=False)
    return True

if __name__ == "__main__":
    print main()

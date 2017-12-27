#!/usr/bin/env python
from PIL import Image
import cv2
from matplotlib import pyplot as plt

def hazarduous():
    # Select and start Camera.
    DEVICE_NO = 0
    capture = cv2.VideoCapture(DEVICE_NO)
    if not capture:
        print "Failed to open camera number %d" %(DEVICE_NO)
        return False
    template = cv2.imread('images/hazard.png',0)
    w, h = template.shape[::-1]
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
        img=gray
        image = Image.fromarray(gray)
        width, height = image.size
        res = cv2.matchTemplate(img,template,cv2.TM_CCOEFF)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
        top_left = max_loc
        bottom_right = (top_left[0] + w, top_left[1] + h)
        print top_left
        cv2.rectangle(img,top_left, bottom_right, 255, 2)

        plt.subplot(121),plt.imshow(res,cmap = 'gray')
        plt.title('Matching Result'), plt.xticks([]), plt.yticks([])
        plt.subplot(122),plt.imshow(img,cmap = 'gray')
        plt.title('Detected Point'), plt.xticks([]), plt.yticks([])
        plt.suptitle("Lesse")
hazarduous()

#!/usr/bin/env python

import math
import cv2

from PIL import Image

def vector(p1, p2):
    return [a-b for a,b in zip (p1,p2)]

def dotproduct(v1, v2):
    return sum((a*b) for a, b in zip(v1, v2))

def length(v):
    return math.sqrt(dotproduct(v, v))

def angle(v1, v2):
    return  math.acos(abs(dotproduct(v1, v2) / (length(v1) * length(v2))))
def isEquilateral(angles):
    flag = True
    for a in angles:
        flag = flag and a > 50 and a < 70
    return flag
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
        img=frame
        image = Image.fromarray(gray)
        width, height = image.size

        ret,thresh = cv2.threshold(gray,100,255,1)
        cv2.imshow('threshold', thresh)
        _,contours,h = cv2.findContours(thresh,1,2)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
#            print len(approx)
            if len(approx)==3 and area > 100:
#                print "triangle"
#                print cnt[0]
#                print 'area =', area
#                print approx[0][0].tolist()
                pts = [approx[i][0].astype(int).tolist() for i in range(3)]
#                print pts
                vectors = [vector(pts[i],pts[(i+1)%3]) for i in range(3)]
#                print vectors
                angles = [ angle(vectors[i],vectors[(i+1)%3]) for i in range(3)]
#                print ans
                angles = [ k*180/math.pi for k in angles]
#                print ans
                if isEquilateral(angles):
                    cv2.drawContours(img,[approx],0,(0,255,0),-1)

        cv2.imshow('img',img)
main()

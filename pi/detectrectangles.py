#!/usr/bin/env python
import cv2
import math
import numpy as np

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

class Rectangle:
    thresh_area = 100.0
    thresh = None
    max_area =-float('inf')
    location = None
    contour = None
    def __init__(self,thresh_area,thresh):
        self.thresh=thresh
        self.thresh_area=thresh_area
    def has_rectangle(self, img=None, DEBUG=False):
        # ret,thresh = cv2.threshold(self.thresh,100,255,1)
        if DEBUG:
            cv2.imshow('threshold', self.thresh)
        ## Changed to CHAIN_APPROX_NONE for all points inside contour
        _,contours,h = cv2.findContours(self.thresh,1, cv2.CHAIN_APPROX_NONE)
        if not contours:
            return False
        cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(cnt)
        approx = cv2.approxPolyDP(cnt,0.001*cv2.arcLength(cnt,True),True)
#            print len(approx)
        if len(approx)>=4 and area > self.thresh_area and area > self.max_area:
#                print "triangle"
#                print cnt[0]
#                print 'area =', area
#                print approx[0][0].tolist()
#                 pts = [approx[i][0].astype(int).tolist() for i in range(3)]
# #                print pts
#                 vectors = [vector(pts[i],pts[(i+1)%3]) for i in range(3)]
# #                print vectors
#                 angles = [ angle(vectors[i],vectors[(i+1)%3]) for i in range(3)]
# #                print ans
#                 angles = [ k*180/math.pi for k in angles]
#                print ans
            # if isEquilateral(angles):
            #     self.max_area=area
            if img is not None:
                cv2.drawContours(img,[approx],0,(0,255,0),-1)
                cv2.imshow('triangle',img)
            self.location=np.array([a[0] for a in approx])
            self.contour=cnt
        if self.location is not None:
            return True
        else:
            return False

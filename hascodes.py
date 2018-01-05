#!/usr/bin/env python

import zbar

from PIL import Image
import cv2
import math

import hascodes

def distance(a,b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def getArea(points):
    pts = [ [a,b] for a,b in points]
    sides=[ distance(pts[i%4],pts[(i+1)%4]) for i in xrange(4) ]
    print sides, pts

class GetCodes:
    hasQR = False
    hasBar = False
    others = False
    qr = []
    bar = []
    gray = None
    location = None
    def clear(self):
        self.hasQR = False
        self.hasBar = False
        self.others = False
        self.qr = []
        self.bar = []
        self.gray = None
        self.location = None
    def __init__(self):
        self.hasQR = False
        self.hasBar = False
        self.others = False
        self.qr = []
        self.bar = []
        self.gray = None
        self.location = None
    def scan(self, gray):
        self.gray = gray
        image = Image.fromarray(gray)
        width, height = image.size
        zbar_image = zbar.Image(width, height, 'Y800', image.tobytes())
        scanner = zbar.ImageScanner()
        scanner.scan(zbar_image)
        cnt=0
        for decoded in zbar_image:
            cnt+=1
            self.location = decoded.location
            print getArea(self.location)
            if decoded.type is zbar.Symbol.EAN13:
                self.hasBar=True
                self.bar.append(decoded)
            elif decoded.type is zbar.Symbol.QRCODE:
                self.hasQR=True
                self.qr.append(decoded)
            else:
                self.others=True
        print cnt

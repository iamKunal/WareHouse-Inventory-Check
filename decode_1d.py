#!/usr/bin/env python
import zbar

from PIL import Image
import cv2

frame = cv2.imread('extra/barcode.png')

gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

image = Image.fromarray(gray)
width, height = image.size
zbar_image = zbar.Image(width, height, 'Y800', image.tobytes())

scanner = zbar.ImageScanner()
scanner.scan(zbar_image)
for decoded in zbar_image:
    print(decoded.data)

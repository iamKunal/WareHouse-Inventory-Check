#!/usr/bin/env python

# import urllib
# from PIL import Image
import cv2
import numpy as np
import array
import subprocess as sp

# import sys

# sys.path.append('./modules')

# import get_yellow
# import detectrectangles


ncCmd=['nc', '-lkp', '5000']
stream = sp.Popen(ncCmd, stdout = sp.PIPE) #, universal_newlines = True)

ffmpegCmd = ['ffmpeg', '-i', '-', '-f', 'rawvideo', '-vcodec', 'bmp',  '-'] #, '-vcodec', 'bmp', ]
ffmpeg = sp.Popen(ffmpegCmd, stdin = stream.stdout, stdout = sp.PIPE)

def main():
    # # Select and start Camera.
    # DEVICE_NO = 0
    # capture = cv2.VideoCapture(DEVICE_NO)
    # if not capture:
    #     print "Failed to open camera number %d" %(DEVICE_NO)
    #     return False
    # i=0
    while True:
        # i+=1
        fileSizeBytes = ffmpeg.stdout.read(6)
        fileSize = 0
        for i in xrange(4):
            fileSize += array.array('B',fileSizeBytes[i + 2])[0] * 256 ** i
        bmpData = fileSizeBytes + ffmpeg.stdout.read(fileSize - 6)
        # if(i%2==0):
        #     continue
        image = cv2.imdecode(np.fromstring(bmpData, dtype = np.uint8), 1)

        frame = image
        # ret, frame = capture.read()
        # if ret==False:
        #     print "Error !"
        #     return False
        ## Gaussian Blur for better 
        # frame = cv2.GaussianBlur(frame,(15,15),0)
        # line = get_yellow.get_frame(frame, DEBUG=False)
# #        cv2.imshow("yellow", get_yellow.get_frame(frame))
#         # cv2.imshow('Canny', cv2.Canny(line,100,200))
#         canny = cv2.Canny(line,100,200)
#         rectangle = detectrectangles.Rectangle(100, line)
#         rectangle.has_rectangle(img=frame, DEBUG=True)
#         if rectangle.contour is not None:
#             ## Centroid of the area
#             M = cv2.moments(rectangle.contour)
#             cx = int(M['m10']/M['m00'])
#             cy = int(M['m01']/M['m00'])
#             cv2.circle(frame, (cx,cy), 10, (0,0,225), 5)
            
#             ## Get all points inside the yellow line
#             cnt=np.array([a[0] for a in rectangle.contour])
#             ## Get line of best fit
#             [vx,vy,x,y] = cv2.fitLine(cnt, cv2.DIST_L2, 0, 0.01, 0.01)
#             ## Get angle between this line and vertical axes.
#             angle = 180*detectrectangles.angle([0,1], [vx,vy])/np.pi
#             turn='right'
#             ## Representation of vx, vy when drone is facing respective direction:
#             ## (++) F (+-)
#             ## --L--+--R--
#             ## (+-) B (++)
            
#             if(vy<0):
#                 turn='left'
#             print 'turn',turn,angle,'degrees.'
#             ## Plot our line on the image.
#             lefty = int((-x*vy/vx) + y)
#             righty = int(((frame.shape[1]-x)*vy/vx)+y)
#             cv2.line(frame,(frame.shape[1]-1,righty),(0,lefty),255,2)
        cv2.imshow('Video Capture', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):       #q to quit
            cv2.destroyAllWindows()
            break
    cv2.destroyAllWindows()
    return True

if __name__ == "__main__":
    print main()

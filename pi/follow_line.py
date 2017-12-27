#!/usr/bin/env python

from PIL import Image
import cv2
import get_yellow
import numpy as np
import time
import detectrectangles

from imutils.video.pivideostream import PiVideoStream
from pi_camera_settings import camera_settings



def main():
    # Select and start Camera.
    vs = PiVideoStream(camera_settings['resolution'],camera_settings['fps']).start()
    time.sleep(2)
    fpscnt = 0
    strt = time.time()
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):       #q to quit
            cv2.destroyAllWindows()
            break

        frame = cv2.flip(vs.read(),camera_settings['flip'])
        fpscnt+=1
        if frame is None:
            print "Error !"
            return False
        line = get_yellow.get_frame(frame, DEBUG=True)
        # cv2.imshow("yellow", get_yellow.get_frame(frame))
        # cv2.imshow('Canny', cv2.Canny(line,100,200))
        # canny = cv2.Canny(line,100,200)
        kernel = np.ones((3,3),np.uint8)
        line = cv2.morphologyEx(line, cv2.MORPH_OPEN, kernel)
        line =  cv2.GaussianBlur(line,(5,5),0)
        rectangle = detectrectangles.Rectangle(100, line)
        rectangle.has_rectangle(img=frame, DEBUG=True)
        dst = cv2.cornerHarris(line,2,3,0.04)
        #----result is dilated for marking the corners, not important-------------
        dst = cv2.dilate(dst,None) 
        #----Threshold for an optimal value, it may vary depending on the image---
        img=frame
        img[dst>0.01*dst.max()]=[0,0,255]
        if rectangle.contour is not None:
            ## Centroid of the area
            # M = cv2.moments(rectangle.contour)
            # cx = int(M['m10']/M['m00'])
            # cy = int(M['m01']/M['m00'])
            # cv2.circle(frame, (cx,cy), 10, (0,0,225), 5)
            
            ## Get all points inside the yellow line
            cnt=np.array([a[0] for a in rectangle.contour])
            ## Get line of best fit
            [vx,vy,x,y] = cv2.fitLine(cnt, cv2.DIST_L2, 0, 0.01, 0.01)
            ## Get angle between this line and vertical axes.
            angle = 180*detectrectangles.angle([0,1], [vx,vy])/np.pi
            turn='left'
            ## Representation of vx, vy when drone is facing respective direction (before flipping):
            ## (++) F (+-)
            ## --L--+--R--
            ## (+-) B (++)
            
            if(vy<0):
                turn='right'
            print 'turn',turn,angle,'degrees.'
            ## Plot our line on the image.
            lefty = int((-x*vy/vx) + y)
            righty = int(((frame.shape[1]-x)*vy/vx)+y)
            cv2.line(frame,(frame.shape[1]-1,righty),(0,lefty),255,2)
        cv2.imshow('Video Capture', frame)
        cv2.imshow('trial_corener', img)
    print "[*]FPS =",fpscnt/(time.time()-strt)
    return True

if __name__ == "__main__":
    print main()

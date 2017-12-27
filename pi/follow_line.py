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
        line = get_yellow.get_frame(frame, DEBUG=True)
        # cv2.imshow("yellow", get_yellow.get_frame(frame))
        # cv2.imshow('Canny', cv2.Canny(line,100,200))
        canny = cv2.Canny(line,100,200)
        rectangle = detectrectangles.Rectangle(100, line)
        rectangle.has_rectangle(img=frame, DEBUG=True)
        if rectangle.contour is not None:
            ## Centroid of the area
            M = cv2.moments(rectangle.contour)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(frame, (cx,cy), 10, (0,0,225), 5)
            
            ## Get all points inside the yellow line
            cnt=np.array([a[0] for a in rectangle.contour])
            ## Get line of best fit
            [vx,vy,x,y] = cv2.fitLine(cnt, cv2.DIST_L2, 0, 0.01, 0.01)
            ## Get angle between this line and vertical axes.
            angle = 180*detectrectangles.angle([0,1], [vx,vy])/np.pi
            turn='right'
            ## Representation of vx, vy when drone is facing respective direction:
            ## (++) F (+-)
            ## --L--+--R--
            ## (+-) B (++)
            
            if(vy<0):
                turn='left'
            print 'turn',turn,angle,'degrees.'
            ## Plot our line on the image.
            lefty = int((-x*vy/vx) + y)
            righty = int(((frame.shape[1]-x)*vy/vx)+y)
            cv2.line(frame,(frame.shape[1]-1,righty),(0,lefty),255,2)
    print "[*]FPS =",cnt/(time.time()-strt)
    return True

if __name__ == "__main__":
    print main()

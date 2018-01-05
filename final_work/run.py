#!/usr/bin/env python

from PIL import Image
import cv2
import numpy as np
import sys
import argparse
from imutils.video.pivideostream import PiVideoStream
from pi_camera_settings import camera_settings


import get_yellow
import detectrectangles
import no_guided
import has_codes


FRAME_AREA = camera_settings['resolution'][0] * camera_settings['resolution'][1]

INITIAL_TARGET_ALTITUDE=2
THRESHOLD_LANDING_AREA=FRAME_AREA * 0.5
THRESHOLD_LANDING_TILT=20
THRESHOLD_TURN_TILT = 60

THRESH_QR_AREA = FRAME_AREA * 

vs = None #Pi Video Stream
capture  = None #Webcam Video Stream



def phase1():
	no_guided.arm_and_takeoff_nogps(INITIAL_TARGET_ALTITUDE)
	while True:				#Loop for getting Yellow Strip to follow
		if cv2.waitKey(1) & 0xFF == ord('q'):       #q to quit
		    cv2.destroyAllWindows()
		    break
		path_frame = cv2.flip(vs.read(),camera_settings['flip'])

	    if path_frame is None:
	        print "Error !"
	        return False
	    no_guided.strafe("forward", duration=0)

	    path_frame = cv2.GaussianBlur(path_frame,(15,15),0)
        line = get_yellow.get_frame(path_frame, DEBUG=False)

	    rectangle = detectrectangles.Rectangle(100, line)
	    rectangle.has_rectangle(img=path_frame, DEBUG=True)
	    
	    angle=None
	    if rectangle.contour is not None:
	        # ## Centroid of the area
	        # M = cv2.moments(rectangle.contour)
	        # cx = int(M['m10']/M['m00'])
	        # cy = int(M['m01']/M['m00'])
	        # cv2.circle(path_frame, (cx,cy), 10, (0,0,225), 5)
	        
	        ## Get all points inside the yellow line
	        cnt=np.array([a[0] for a in rectangle.contour])
	        ## Get line of best fit
	        [vx,vy,x,y] = cv2.fitLine(cnt, cv2.DIST_L2, 0, 0.01, 0.01)
	        ## Get angle between this line and vertical axes.
	        angle = 180*detectrectangles.angle([0,1], [vx,vy])/np.pi
	        # turn='right'
	        # ## Representation of vx, vy when drone is facing respective direction:
	        # ## (++) F (+-)
	        # ## --L--+--R--
	        # ## (+-) B (++)
	        
	        # if(vy<0):
	        #     turn='left'
	        # print 'turn',turn,angle,'degrees.'
	        # ## Plot our line on the image.
	        # lefty = int((-x*vy/vx) + y)
	        # righty = int(((path_frame.shape[1]-x)*vy/vx)+y)
	        # cv2.line(frame,(path_frame.shape[1]-1,righty),(0,lefty),255,2)
	    if angle < THRESHOLD_LANDING_TILT and cv2.contourArea(rectangle.contour) < THRESHOLD_LANDING_AREA:		#If tilt is less than 20 and take-off area is not visible
	    	#Successfully Reached the Yellow line
	    	break
def scan_phase():
	QRCODE = None
	Triangle_Present = False
	while True:				#Loop for getting Yellow Strip to follow
		if cv2.waitKey(1) & 0xFF == ord('q'):       #q to quit
		    cv2.destroyAllWindows()
		    break
		ret, frame = capture.read()
        if ret==False:
            print "Error !"
            return False
        cv2.imshow('Video Capture', frame)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	    no_guided.strafe("left", duration=0)

		ret,thresh = cv2.threshold(gray,127,255,1)
		cv2.imshow('Gray', np.hstack([gray,thresh]))
		codes = hascodes.GetCodes()
		codes.scan(gray)
		current_area=None
		if codes.hasQR:
		   print len(codes.qr)
		   for c in codes.qr:
		       print c.data
		       current_area=codes.area
		if codes.hasBar:
		   print len(codes.bar)
		   for c in codes.bar:
		       print c.data
		print "Frame Done"
		triangle = detecttriangle.DetectTriangle(100, gray)
		triangle_present = triangle.has_triangle(frame)
		triangle_location = triangle.location
		if codes.hasQR and triangle_present:
			QRCODE = codes.qr[0].data
			Triangle_Present = True
			break
		if current_area > THRESH_QR_AREA:
			QRCODE = codes.qr[0].data
			break





	while True:				#Loop for getting Yellow Strip to follow
		if cv2.waitKey(1) & 0xFF == ord('q'):       #q to quit
		    cv2.destroyAllWindows()
		    break
		path_frame = cv2.flip(vs.read(),camera_settings['flip'])

	    if path_frame is None:
	        print "Error !"
	        return False

	    path_frame = cv2.GaussianBlur(path_frame,(15,15),0)
        line = get_yellow.get_frame(path_frame, DEBUG=False)

	    rectangle = detectrectangles.Rectangle(100, line)
	    rectangle.has_rectangle(img=path_frame, DEBUG=True)
	    
	    tilt_angle,angle=None
	    if rectangle.contour is not None:
	        # ## Centroid of the area
	        # M = cv2.moments(rectangle.contour)
	        # cx = int(M['m10']/M['m00'])
	        # cy = int(M['m01']/M['m00'])
	        # cv2.circle(path_frame, (cx,cy), 10, (0,0,225), 5)
	        
	        ## Get all points inside the yellow line
	        cnt=np.array([a[0] for a in rectangle.contour])
	        ## Get line of best fit
	        [vx,vy,x,y] = cv2.fitLine(cnt, cv2.DIST_L2, 0, 0.01, 0.01)
	        ## Get angle between this line and vertical axes.
	        angle = 180*detectrectangles.angle([0,1], [vx,vy])/np.pi
	        # turn='right'
	        # ## Representation of vx, vy when drone is facing respective direction:
	        # ## (++) F (+-)
	        # ## --L--+--R--
	        # ## (+-) B (++)
	        
	        # if(vy<0):
	        #     turn='left'
	        # print 'turn',turn,angle,'degrees.'
	        # ## Plot our line on the image.
	        # lefty = int((-x*vy/vx) + y)
	        # righty = int(((path_frame.shape[1]-x)*vy/vx)+y)
	        # cv2.line(frame,(path_frame.shape[1]-1,righty),(0,lefty),255,2)
	    	if vy>0:
                tilt_angle=-angle
            else:
            	tilt_angle=angle
	    rows,cols = line.shape
	    M = cv2.getRotationMatrix2D((cols/2,rows/2),tilt_angle,1)
	    rotated = cv2.warpAffine(line,M,(cols,rows))
	    sobely = cv2.Sobel(rotated,cv2.CV_64F,0,1,ksize=1)

	    kernel = np.ones((3,3),np.uint8)
	    # sobely = cv2.morphologyEx(sobely, cv2.MORPH_OPEN, kernel)
	    minLineLength=100
	    sobely=cv2.convertScaleAbs(cv2.GaussianBlur(sobely,(5,5),0))

	    ret,sobely = cv2.threshold(sobely,100,255,cv2.THRESH_BINARY)
	    try:
	        edges = cv2.HoughLinesP(image=sobely,rho=0.1,theta=1*np.pi/2, threshold=50,lines=np.array([]), minLineLength=minLineLength,maxLineGap=100)
	    
	        a,b,c = edges.shape
	        for i in range(1):
	            cv2.line(path_frame, (edges[i][0][0], edges[i][0][1]), (edges[i][0][2], edges[i][0][3]), (0, 0, 255), 3, cv2.LINE_AA)
	    	print "Strip Found, going left for scanning"
	    	scan_phase()

	    except:
	        print "Strip not found"
	    # cv2.imshow("rotate", sobely)
	    # cv2.imshow('Video Capture', path_frame)

	    if angle >= THRESHOLD_TURN_TILT:
	    	print "Right Turn is found."
	    	break

def phase2():
	while True:				#Loop for getting Yellow Strip to follow
		if cv2.waitKey(1) & 0xFF == ord('q'):       #q to quit
		    cv2.destroyAllWindows()
		    break
		path_frame = cv2.flip(vs.read(),camera_settings['flip'])

	    if path_frame is None:
	        print "Error !"
	        return False

	    path_frame = cv2.GaussianBlur(path_frame,(15,15),0)
        line = get_yellow.get_frame(path_frame, DEBUG=False)

	    rectangle = detectrectangles.Rectangle(100, line)
	    rectangle.has_rectangle(img=path_frame, DEBUG=True)
	    
	    tilt_angle,angle=None
	    if rectangle.contour is not None:
	        # ## Centroid of the area
	        # M = cv2.moments(rectangle.contour)
	        # cx = int(M['m10']/M['m00'])
	        # cy = int(M['m01']/M['m00'])
	        # cv2.circle(path_frame, (cx,cy), 10, (0,0,225), 5)
	        
	        ## Get all points inside the yellow line
	        cnt=np.array([a[0] for a in rectangle.contour])
	        ## Get line of best fit
	        [vx,vy,x,y] = cv2.fitLine(cnt, cv2.DIST_L2, 0, 0.01, 0.01)
	        ## Get angle between this line and vertical axes.
	        angle = 180*detectrectangles.angle([0,1], [vx,vy])/np.pi
	        # turn='right'
	        # ## Representation of vx, vy when drone is facing respective direction:
	        # ## (++) F (+-)
	        # ## --L--+--R--
	        # ## (+-) B (++)
	        
	        # if(vy<0):
	        #     turn='left'
	        # print 'turn',turn,angle,'degrees.'
	        # ## Plot our line on the image.
	        # lefty = int((-x*vy/vx) + y)
	        # righty = int(((path_frame.shape[1]-x)*vy/vx)+y)
	        # cv2.line(frame,(path_frame.shape[1]-1,righty),(0,lefty),255,2)
	    	if vy>0:
                tilt_angle=-angle
            else:
            	tilt_angle=angle
	    rows,cols = line.shape
	    M = cv2.getRotationMatrix2D((cols/2,rows/2),tilt_angle,1)
	    rotated = cv2.warpAffine(line,M,(cols,rows))
	    sobely = cv2.Sobel(rotated,cv2.CV_64F,0,1,ksize=1)

	    kernel = np.ones((3,3),np.uint8)
	    # sobely = cv2.morphologyEx(sobely, cv2.MORPH_OPEN, kernel)
	    minLineLength=100
	    sobely=cv2.convertScaleAbs(cv2.GaussianBlur(sobely,(5,5),0))

	    ret,sobely = cv2.threshold(sobely,100,255,cv2.THRESH_BINARY)
	    try:
	        edges = cv2.HoughLinesP(image=sobely,rho=0.1,theta=1*np.pi/2, threshold=50,lines=np.array([]), minLineLength=minLineLength,maxLineGap=100)
	    
	        a,b,c = edges.shape
	        for i in range(1):
	            cv2.line(path_frame, (edges[i][0][0], edges[i][0][1]), (edges[i][0][2], edges[i][0][3]), (0, 0, 255), 3, cv2.LINE_AA)
	    	print "Strip Found, going left for scanning"

	    except:
	        print "Strip not found"
	    # cv2.imshow("rotate", sobely)
	    # cv2.imshow('Video Capture', path_frame)

	    if angle >= THRESHOLD_TURN_TILT:
	    	print "Right Turn is found."
	    	break

def main():
	parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
	parser.add_argument('--connect', 
	                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
	args = parser.parse_args()

	connection_string = args.connect
	sitl = None

	connection_string = '127.0.0.1:14552'

	#Start SITL if no connection string specified
	if not connection_string:
		print "Please input connection string"
		return False

	# Connect to the Vehicle
	print('Connecting to vehicle on: %s' % connection_string)
	no_guided.connect_to_vehicle(connection_string)

    # Startup PiCamera.
    vs = PiVideoStream(camera_settings['resolution'],camera_settings['fps']).start()
    time.sleep(2)

    # Select and start WebCam.
    DEVICE_NO = 0
    capture = cv2.VideoCapture(DEVICE_NO)
    if not capture:
        print "Failed to open camera number %d" %(DEVICE_NO)
        return False
    
     '''
		Phase 1  is Take-off and moving forward until detection of yellow line.

		Phase 2 is moving until right turn is found.

		Phase 3 is taking the right turn and moving until Landing area is found.

		Phase 4 is landing on the yellow area.

		Scan_Phase is used to strafe copter for scanning the QR Codes.

     '''


    phase1()		#Takeoff and detect the Yellow Line


    

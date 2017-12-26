import array
import subprocess as sp
import numpy as np
import cv2
import time

WIDTH,HEIGHT=[600,400]

ncCmd=['nc', '-lkp', '5000']
stream = sp.Popen(ncCmd, stdout = sp.PIPE) #, universal_newlines = True)

ffmpegCmd = ['ffmpeg', '-i', '-', '-f', 'rawvideo', '-vcodec', 'bmp',  '-'] #, '-vcodec', 'bmp', ]
ffmpegCmd = ['ffmpeg',
            '-i', '-',
            '-f', 'rawvideo',
            # '-f', 'image2pipe',
            '-pix_fmt', 'rgb24',
            '-vcodec', 'rawvideo', '-']
ffmpeg = sp.Popen(ffmpegCmd, stdin = stream.stdout, stdout = sp.PIPE)



# cap = cv2.VideoCapture('/dev/stdin')

# time.sleep(1)
i=0
while True:
	raw_image = ffmpeg.stdout.read(WIDTH*HEIGHT*3)
	# i=i%2+1
	# if i%2:
	# 	continue
	# image=cv2.imdecode(np.fromstring(raw_image, dtype='uint8'),cv2.CV_LOAD_IMAGE_COLOR)
	# print len(raw_image)
	# break
	image=np.fromstring(raw_image, dtype='uint8').reshape((HEIGHT,WIDTH,3))
	# img2=np.array(image)
	image=cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
	print image
	# break
	cv2.imshow("im",image)
	# print type(image) 
	# cv2.waitKey(25)
	# print "H"
	# # ret, frame = cap.read()
	# fileSizeBytes = ffmpeg.stdout.read(6)
	# fileSize = 0
	# for i in xrange(4):
	# 	fileSize += fileSizeBytes[i + 2] * 256 ** i
	# bmpData = fileSizeBytes + ffmpeg.stdout.read(fileSize - 6)
	# frame = cv2.imdecode(np.fromstring(bmpData, dtype = np.uint8), 1)
	# gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	# cv2.imshow('frame',gray)
	# time.sleep(10)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# cap.release()
cv2.destroyAllWindows()
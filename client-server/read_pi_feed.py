import array
import subprocess as sp
import numpy as np
import cv2
import time


ncCmd=['nc', '-lkp', '5000']
stream = sp.Popen(ncCmd, stdout = sp.PIPE) #, universal_newlines = True)

ffmpegCmd = ['ffmpeg', '-i', '-', '-f', 'rawvideo', '-vcodec', 'bmp',  '-'] #, '-vcodec', 'bmp', ]
ffmpeg = sp.Popen(ffmpegCmd, stdin = stream.stdout, stdout = sp.PIPE)


while True:
	fileSizeBytes = ffmpeg.stdout.read(6)
	fileSize = 0
	for i in xrange(4):
		fileSize += array.array('B',fileSizeBytes[i + 2])[0] * 256 ** i
	bmpData = fileSizeBytes + ffmpeg.stdout.read(fileSize - 6)
	image = cv2.imdecode(np.fromstring(bmpData, dtype = np.uint8), 1)

	cv2.imshow("im",image)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break


cv2.destroyAllWindows()
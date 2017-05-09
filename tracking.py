from __future__ import division

import math
import numpy as np

import imutils
import cv2

import serial


Lower = (29, 86, 6)
Upper = (64, 255, 255)

global resol
resol = (900,700)



camera = cv2.VideoCapture(0)

ser = serial.Serial ("/dev/ttyS0")    #Open named port 
ser.baudrate = 9600	                     #Set baud rate to 9600
while True:
		
	
	_,frame = camera.read()
	#frame = cv2.imread("marker.png")
	
  
	frame = imutils.resize(frame, width = resol[0], height = resol[1])
	
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

		
	mask = cv2.inRange(hsv, Lower, Upper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
				cv2.CHAIN_APPROX_SIMPLE)[-2]
	center_array = []
	center_array1 = []
	center = None
	cv2.circle(frame, (int(resol[0]/2),int(resol[1]/2)), 5, (255, 255, 255), -1)
	for c in cnts:
		if len(cnts) > 0:
						
			
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		 
			if radius > 10:
				cv2.circle(frame, (int(x), int(y)), int(radius),
						(0, 255, 255), 2)
				cv2.circle(frame, center, 5, (0, 0, 255), -1)
				center_array += [center]
				center_array1 += [[[int(x),int(y)]]]

		
	try:
			
		 pt1,pt2,pt3,pt4 = center_array[0],center_array[1],center_array[2],center_array[3]
			
								
		 midx1 = (pt1[0] + pt2[0])/2
		 midy1 = (pt1[1] + pt3[1])/2

		 midx2 = (pt4[0] + pt3[0])/2
		 midy2 = (pt4[1] + pt2[1])/2

		 mid = int((midx1 + midx2)/2),int((midy1 + midy2)/2)
		 
		 line_center = (int(mid[0]/2+resol[0]/4), int(mid[1]/2+resol[1]/4))

		 '''=========================================================================='''
		 # codist will give you values in co-orinates
		 # dist will give you the distance to center

		 dist = math.sqrt(math.pow(resol[1]/2-mid[1],2)+math.pow(resol[0]/2-mid[0],2))
		 codist = (mid[0]-resol[0]/2),-(mid[1]-resol[1]/2)
		 '''=========================================================================='''

		 print 'dist:',dist
		 print 'co-ordinate_distance',codist

		 cv2.line(frame,mid,(int(resol[0]/2),int(resol[1]/2)),(0, 0, 255),2)
		 cv2.circle(frame, mid, int(radius),
						(0, 255, 255), 2)
		 cv2.circle(frame, mid, 5, (0, 0, 255), -1)

		 cv2.putText(frame, str(dist)+','+str(codist) ,line_center , cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
		
		 ser.write(str(dist)+','+str( (mid[0]-resol[0]/2))+','+str( -(mid[1]-resol[1]/2))+'\n')                         #Send data to arduino
		 


	except:

		print '====== Error ======='
		print len(center_array) , 'points '
		print center_array1
		print '===================='
		
						
						
				
				
		# show the frame to screen
	cv2.imshow("Frame", frame)
	cv2.imshow("Mask", mask)
	

	key = cv2.waitKey(1) & 0xFF
		 
				
	if key == 27:
		break

ser.close()     
	
	

camera.release()
cv2.destroyAllWindows()


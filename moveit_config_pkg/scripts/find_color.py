#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import sys
import cv2
import numpy as np


cap=cv2.VideoCapture(0)


pub = rospy.Publisher('direction', String, queue_size=10)
rospy.init_node('vision', anonymous=True)
rate = rospy.Rate(10) # 10hz
rospy.loginfo("Direction is published ........ # see /direction topic")


x_d=0.0
y_d=0.0
x_d_p=0.0
y_d_p=0.0
message = "0"

while(1):
	_, img = cap.read()
	    
	#converting frame(img i.e BGR) to HSV (hue-saturation-value)

	hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
	blue_lower=np.array([94,123,46],np.uint8)
	blue_upper=np.array([125,255,255],np.uint8)


	blue=cv2.inRange(hsv,blue_lower,blue_upper)
	
	#Morphological transformation, Dilation  	
	kernal = np.ones((5 ,5), "uint8")


	blue=cv2.dilate(blue,kernal)


	(_,contours,hierarchy)=cv2.findContours(blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	
	if len(contours)>0:
		contour= max(contours,key=cv2.contourArea)
		area = cv2.contourArea(contour)
		if area>800: 
			x,y,w,h = cv2.boundingRect(contour)
		
			ss = 'x:' + str(x) + ', y:' + str(y)
			
			cv2.putText(img,ss,(x-20,y-5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),1,cv2.LINE_AA)
			
			if x>0 and x<160:
				if y>0 and y<120:
					message = "1"
				elif y>360 and y<480:
					message = "3"
				else:
					message = "0"

			elif x>480 and x<640:
				if y>0 and y<120:
					message = "2"
				elif y>360 and y<480:
					message = "4"
				else:
					message = "0"

			else:
				message = "0"

		
	
	cv2.imshow("Mask",blue)
	cv2.imshow("Color Tracking",img)
	k = cv2.waitKey(10) & 0xff # Press 'ESC' for exiting video
    	if k == 27:
		break

	
	if not rospy.is_shutdown():
		direc_str = message
		pub.publish(direc_str)
		rate.sleep()

cap.release()
cv2.destroyAllWindows()


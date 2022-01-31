#! /usr/bin/env python
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int16

bridge = CvBridge()
FT = 0
l = 60 
x_ref = 120
x1 = 120
x2 = 120
x1_h = 120
x2_h = 120

h_vis =1.0/30.0
e_y_1 = 0.0
ie_y = 0.0
u = 90
v = -1200 #-800 #-1200

#******************************************************************************************}
#******************************************************************************************
#******************************************************************************************
def tip(imagenN):
	H=np.array([[-7.98362236e-02,-4.79765416e-01,1.23982766e+02],[1.05493081e-03,-1.61957424,3.77026220e+02],[7.48177877e-06,-4.86995945e-03,1.0]]) 
	imagenH = cv2.warpPerspective(imagenN, H, (200,300),borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0)) 
	return imagenH
#******************************************************************************************}
#******************************************************************************************
#******************************************************************************************
def roi_zone(x):
	assert (x>=0) and (x<=199), 'x out of limits'
	if (x>130) and (x<=199):
		y = int(round(-1.6875*x+499.8125))
	if (x>=69) and (x<=130):
		y = 280
	if (x>=0) and (x<69):
		y = int(round(1.7375*x+160.0))
	return y
#******************************************************************************************}
#******************************************************************************************
#******************************************************************************************
def vec_create(x,stride):
	j = 0
	xv = []
	for i in range(0,2*stride+1):
		if ((-1)**i==-1): j = j+1
		xv.append(x+j*(-1)**i)
	return xv
#******************************************************************************************}
#******************************************************************************************
#******************************************************************************************
def line_detector(imagen0,x1,l,side):
	K = True
	stride = 6
	y1 = roi_zone(x1)
	x1v = vec_create(x1,stride)
	while (K==True):
		if (y1+stride>280): m = 280-y1
		else: m = stride
		for j in range(y1+m,y1-stride,-1):
			for i in x1v:
				if imagen0[j][i]==255:
					x1 = i
					y1 = j
					K = False
					break
			x1v = vec_create(x1,stride)
			if (K==False): break
		if (K==True): 
			x1 = x1-1*side
			y1 = roi_zone(x1)
	x2 = x1
	x2v = vec_create(x2,stride)
	for j in range(y1-1,y1-l,-1):
		for i in x2v:
			if imagen0[j][i]==255:
				x2 = i
				y2 = j
				K = False
				break
		x2v = vec_create(x2,stride)			
	return x1,y1,x2,y2
#******************************************************************************************
#******************************************************************************************
#******************************************************************************************
def callback_V(data0):
	global u, v, e_y_1, ie_y
	global FT
	global x1, x2, x1_h, x2_h
	imagen0 = bridge.imgmsg_to_cv2(data0, "bgr8") 	
	imagenG = cv2.cvtColor(imagen0,cv2.COLOR_BGR2GRAY) 					
	imagenT = tip(imagenG)
	_,imagenB = cv2.threshold(imagenT,75,255,cv2.THRESH_BINARY)
	imagenF = cv2.Sobel(imagenB,cv2.CV_8U,1,0, ksize=3)
	y1 = 0
	y2 = 0
	if (FT<=30):
		x1 = 180
		FT = FT+1
	else: 
		x1 = x1_h
	x1,y1,x2,y2 = line_detector(imagenF,x1,l,True)
	x1_h = x1
	x2_h = x2

	# CONTROL
	ky = 0.0909961 
	kth = 0.20056981
	kdy = 0.0075
	# vrpm	R			Q				Ky					Kth					Kdy
	# 800		24		0.04*I	0.0909961 , 0.20056981, 0.0075  +++



	e_y = x1-x_ref
	e_th = np.arctan2(x2-x1,l)
	#ie_y = ie_y+(h_vis/2.0)*(e_y+e_y_1) # Int. Trapezoidal
	de_y = (e_y-e_y_1)/h_vis
	e_y_1 = e_y

	u = int(round(90-np.arctan(ky*e_y+kth*e_th+kdy*de_y)*(180/np.pi))) 
	print('steering ',u)

 	#Visualizacion
	#namedWindow("homografia");
	imagenS = cv2.cvtColor(imagenF,cv2.COLOR_GRAY2BGR)
	imagenS = cv2.circle(imagenS,(x1,y1),3,(0, 0, 255),-1)
	imagenS = cv2.circle(imagenS,(x2,y2),3,(0, 0, 255),-1)
	imagenS = cv2.line(imagenS, (x1,y1), (x2,y2), (0, 0, 255), 2) 
	cv2.imshow('homografia',imagenS)	
	cv2.moveWindow("homografia", 400,20)
	cv2.waitKey(1)

	Vpub.publish(v) 
	Spub.publish(u)
#******************************************************************************************}
#******************************************************************************************
#******************************************************************************************
if __name__ == '__main__':
	print("Equipo: DotMEX-CAR")
	print("Nodo inicializado: TMR_01.py")
	rospy.init_node('TMR_01',anonymous=True)												
	Vpub = rospy.Publisher('/AutoNOMOS_mini/manual_control/speed',Int16,queue_size=15)				 
	Spub = rospy.Publisher('/AutoNOMOS_mini/manual_control/steering',Int16,queue_size=15)
	rospy.Subscriber("/app/camera/rgb/image_raw",Image,callback_V)	 						
	rospy.spin()

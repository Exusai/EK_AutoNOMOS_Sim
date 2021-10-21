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
u = 90
v = -600 

k = 1

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
	global u, v
	global FT
	global x1, x2, x1_h, x2_h
	global k
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

	kx = 0.05616094 
	kth = 0.16484354
	# vrpm	R				Q				Kx						Kth
	# 800		20		0.05*I	0.0487392   0.15340431
	# 800		15		0.05*I	0.05616094	0.16484354
	# 800		10		0.05*I	0.06855525  0.18258912

	ex = x1-x_ref
	th = np.arctan2(x2-x1,l)
	u = int(round(90-np.arctan(kx*ex+kth*th)*(180/np.pi))) 
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

	# Guarda el dataset
	f1 = open('steering.csv','a+')
	f1.write("%5.2f\n" %(u))
	f1.close()
	cv2.imwrite('dataset/im_road'+str(k)+'.png',imagen0)
	k = k+1

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

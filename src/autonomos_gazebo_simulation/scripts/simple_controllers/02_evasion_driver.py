#! /usr/bin/env python
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan, Imu

bridge = CvBridge()

# V. Globales Vision
s = 180
l = 60 
FT = 0
x_ref = 120
x1 = 105
x2 = 120
x1_h = 120
x2_h = 120
th = 0.0

# V. Globales Lidar
Ev = False
Obs1 = False
step = 0
curl = False

# V. Globales Imu
f_imu = 50.0
h = 1.0/f_imu
yaw = 0.0
yaw_h = 0.0
yaw0 = 0.0
Dyaw = 0.0
FTY = True
D = 28.0
d = 0.0
ky = 5.0

# V. Globales Control
h_vis =1.0/30.0
e_y_1 = 0.0
ie_y = 0.0
u = 90
v = -600 #-800 #-1200

#**********************************************************************************************************************************
#**********************************************************************************************************************************
#**********************************************************************************************************************************
def tip(imagenN):
	H=np.array([[-7.98362236e-02,-4.79765416e-01,1.23982766e+02],[1.05493081e-03,-1.61957424,3.77026220e+02],[7.48177877e-06,-4.86995945e-03,1.0]]) 
	imagenH = cv2.warpPerspective(imagenN, H, (200,300),borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0)) 
	return imagenH
#**********************************************************************************************************************************
#**********************************************************************************************************************************
#**********************************************************************************************************************************
def roi_zone(x):
	assert (x>=0) and (x<=199), 'x out of limits'
	if (x>130) and (x<=199):
		y = int(round(-1.6875*x+499.8125))
	if (x>=69) and (x<=130):
		y = 280
	if (x>=0) and (x<69):
		y = int(round(1.7375*x+160.0))
	return y
#**********************************************************************************************************************************
#**********************************************************************************************************************************
#**********************************************************************************************************************************
def vec_create(x,stride):
	j = 0
	xv = []
	for i in range(0,2*stride+1):
		if ((-1)**i==-1): j = j+1
		xv.append(x+j*(-1)**i)
	return xv
#**********************************************************************************************************************************
#**********************************************************************************************************************************
#**********************************************************************************************************************************
def line_detector(imagen0,x1,l,side):
	K = True
	stride = 8
	y1 = roi_zone(x1)
	x1v = vec_create(x1,stride)
	while (K==True):
		if (y1+stride>299): m = 299-y1
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
#**********************************************************************************************************************************
#**********************************************************************************************************************************
#**********************************************************************************************************************************
def callback_imu(data2):
	global yaw0, yaw_h, Dyaw
	gz = data2.angular_velocity.z
	yaw = (yaw_h+h*gz)	
	if (yaw>60*np.pi) or (yaw<-60*np.pi):
		yaw = 0.0
		yaw0 = 0.0
	yaw_h = yaw
	if (FTY==True): yaw0 = yaw
	Dyaw = (yaw0-yaw)*(180.0/np.pi)
#**********************************************************************************************************************************
#**********************************************************************************************************************************
#**********************************************************************************************************************************
def callback_L(data1):
	global Ev,Obs1, step
	global FT, FTY, D, curl, s
	global u,v
	R_full = data1.ranges
	k = 0
	R = []
	for r in R_full:
		if (r>=3.0) or (r<=0.11): r = 3.0
		R.append(r)
		k = k+1
	r_min = np.amin(R) 
	r270 = R[269]
	R0_i = R[0:19] 			
	R0_d = R[339:359] 
	R0 = np.concatenate((R0_i,R0_d))
	r0 = np.amin(R0)
	Rlim = 1.0
	if (r0<=Rlim): Obs1 = True
	else: Obs1 = False
	if (Obs1==True): 
		Ev = True
		FTY = False
	if (Ev == True):
		print('-----------EVASION-----------')
		v = -400
		if (step == 0):
			u = 180
			if (Dyaw<=-D): step = step+1
		if (step == 1):
			u = 0
			if (Dyaw>=-d): step = step+1
		if (step == 2):
			u = 90+ky*Dyaw
			if (r_min>=0.4) and (r270>=0.5):
				Ev = False
				FT = 0
				FTY = True
				s = 160
				step = 0
		if (step == 3):
			u = 0
			if (Dyaw>=-40): step = step+1
		if (step == 4):
			u = 180
			if (Dyaw<=-D): 
				Ev = False
				FT = 0
				FTY = True
				s = 140
				step = 0
		print('step ',step)
		print('Dyaw ',Dyaw)
		Vpub.publish(v) 
		Spub.publish(u)
#**********************************************************************************************************************************
#**********************************************************************************************************************************
#**********************************************************************************************************************************
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
	if (Ev==False):
		if (FT<=30):
			x1 = 105
			FT = FT+1
			v = -500
		else: 
			x1 = x1_h
			v = -700#-800
		side = -1
		x1,y1,x2,y2 = line_detector(imagenF,x1,l,side)
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
#**********************************************************************************************************************************
#**********************************************************************************************************************************
#**********************************************************************************************************************************
if __name__ == '__main__':
	print("Equipo: DotMEX-CAR")
	print("Nodo inicializado: TMR_02.py")			
	rospy.init_node('TMR_02',anonymous=True)								
	Vpub = rospy.Publisher('/AutoNOMOS_mini/manual_control/speed',Int16,queue_size=15)				 
	Spub = rospy.Publisher('/AutoNOMOS_mini/manual_control/steering',Int16,queue_size=15)
	rospy.Subscriber("/app/camera/rgb/image_raw",Image,callback_V)	 						
	rospy.Subscriber('/scan', LaserScan, callback_L)
	rospy.Subscriber('/AutoNOMOS_mini/imu', Imu, callback_imu)	
	rospy.spin()


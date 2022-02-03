#!/usr/bin/env python
import rospy
import rospkg
import math
import numpy as np
from std_msgs.msg import Int16
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import SpawnModelRequest
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
#---------------------------------------------------------------------------------------------------
#---------------------------------------------------------------------------------------------------
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
v = -300#-400, -100
#---------------------------------------------------------------------------------------------------
#---------------------------------------------------------------------------------------------------
rospack  = rospkg.RosPack()
pkg_path = rospack.get_path('autonomos_gazebo_simulation')
spawn_model    = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
spawn_req      = SpawnModelRequest()
spawn_req.model_name = "Dyn_car01"
spawn_req.model_xml  = open(pkg_path + '/models/Dyn_car01/model.sdf','r').read()
spawn_req.robot_namespace = ''
spawn_req.initial_pose.position.x = 6.0
spawn_req.initial_pose.position.y = -23.1299
spawn_req.initial_pose.position.z = 0.0197
spawn_req.initial_pose.orientation.z = 3.1416
spawn_req.initial_pose.orientation.w = 0.0
spawn_model(spawn_req)
#****************************************************************************************************
#****************************************************************************************************
#****************************************************************************************************
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
	imagen0 = bridge.imgmsg_to_cv2(data0, "bgr8") 	
	imagenG = cv2.cvtColor(imagen0,cv2.COLOR_BGR2GRAY) 					
	imagenT = tip(imagenG)
	_,imagenB = cv2.threshold(imagenT,75,255,cv2.THRESH_BINARY)
	imagenF = cv2.Sobel(imagenB,cv2.CV_8U,1,0, ksize=3)
	y1 = 0
	y2 = 0
	if (FT<=90):
		x1 = 180
		FT = FT+1
	else: 
		x1 = x1_h
	x1,y1,x2,y2 = line_detector(imagenF,x1,l,True)
	x1_h = x1
	x2_h = x2
	kx = 0.05616094 
	kth = 0.16484354
	ex = x1-x_ref
	th = np.arctan2(x2-x1,l)
	u = int(round(90-np.arctan(kx*ex+kth*th)*(180/np.pi))) 
	#print('steering ',u)
	"""
 	#Visualizacion
	#namedWindow("homografia");
	imagenS = cv2.cvtColor(imagenF,cv2.COLOR_GRAY2BGR)
	imagenS = cv2.circle(imagenS,(x1,y1),3,(0, 0, 255),-1)
	imagenS = cv2.circle(imagenS,(x2,y2),3,(0, 0, 255),-1)
	imagenS = cv2.line(imagenS, (x1,y1), (x2,y2), (0, 0, 255), 2) 
	cv2.imshow('homografia',imagenS)	
	cv2.moveWindow("homografia", 400,20)
	cv2.waitKey(1)
	"""
	Vpub.publish(v) 
	Spub.publish(u)

#****************************************************************************************************
#****************************************************************************************************
#****************************************************************************************************
if __name__ == "__main__":
	rospy.init_node('roscar_dyn_car',anonymous=True)												
	print("Nodo inicializado: roscar_dyn_car.py")
	Vpub = rospy.Publisher("/Dyn_car01/manual_control/speed",    Int16, queue_size=15)	 
	Spub = rospy.Publisher("/Dyn_car01/manual_control/steering", Int16, queue_size=15)
	rospy.Subscriber("Dyn_car01/camera_obs/rgb/image_raw",Image,callback_V)	 						
	rospy.spin()

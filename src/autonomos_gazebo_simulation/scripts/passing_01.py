#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan#, Imu
from std_msgs.msg import Int16

path_libs = '/home/faber/EK_AutoNOMOS_Sim/src/autonomos_gazebo_simulation/scripts/libs'
import sys
sys.path.insert(1, path_libs)
from line_finder import tip, line_detector

#**********************************************************************************************************************************
#**********************************************************************************************************************************
#**********************************************************************************************************************************
class sensors_processing(object):
#----------------------------------------------------------------------------------------------------------------
#																	INIT
#----------------------------------------------------------------------------------------------------------------
	def __init__(self):
		# V. IMU
		#self.FTY = True
		#self.yaw0 = 0.0
		#self.yaw_h = 0.0
		#self.Dyaw = 0.0
		# V. Camara
		self.imagen0 = np.zeros((480,640))
		self.x1_h = 120
		self.FT = 0
		self.ex = 0
		self.th = 0.0
		self.i = 1
		#V. Lidar
		self.step = 0
		self.R = np.zeros(360)

		#V. Rebase
		self.Ev = False
		self.NV_fcar = 0
		self.V_fcar = 0.0

		rospy.Subscriber("/app/camera/rgb/image_raw",Image,self.callback_Vis)
		rospy.Subscriber('/scan', LaserScan, self.callback_Lidar)
		#rospy.Subscriber('/AutoNOMOS_mini/imu', Imu, self.callback_Imu)
		self.Vpub = rospy.Publisher('/AutoNOMOS_mini/manual_control/speed',Int16,queue_size=15)				 
		self.Spub = rospy.Publisher('/AutoNOMOS_mini/manual_control/steering',Int16,queue_size=15)

#----------------------------------------------------------------------------------------------------------------
#																	CALLBACKS
#----------------------------------------------------------------------------------------------------------------
#																	VISION
	def callback_Vis(self, data_vis):
		bridge = CvBridge()
		self.imagen0 = bridge.imgmsg_to_cv2(data_vis, "bgr8") 	
		self.maneuver()

#----------------------------------------------------------------------------------------------------------------
#																	LIDAR
	def callback_Lidar(self, data_lidar):
		i = 0 
		for r in data_lidar.ranges:
			if (r>=3.0): r = 3.0 
			self.R[i] = r
			i = i+1

#----------------------------------------------------------------------------------------------------------------
#																	IMU
	"""
	def callback_Imu(self, data_imu):
		f_imu = 50.0
		h = 1.0/f_imu
		gz = data_imu.angular_velocity.z
		yaw = (self.yaw_h+h*gz)	
		if (yaw>60*np.pi) or (yaw<-60*np.pi):
			yaw = 0.0
			self.yaw0 = 0.0
		self.yaw_h = yaw
		if (self.FTY==True): self.yaw0 = yaw
		self.Dyaw = (self.yaw0-yaw)*(180.0/np.pi)
	"""		
#----------------------------------------------------------------------------------------------------------------
#																	CONTROLLERS
#----------------------------------------------------------------------------------------------------------------
	def maneuver(self):
		value = 1
		R0_i = self.R[0:15]
		R0_d = self.R[344:359]
		R0 = np.concatenate((R0_i,R0_d))
		r0 = np.amin(R0)
		if (abs(self.th)<=0.0873): self.Curve = False
		else: self.Curve = True
		if (r0<=1.5): 
			self.Ev = True
		if (self.Ev==True): value = 2

		#_______________________________________________________________________________________
		if (value == 1):
			l = 60 
			x_ref = 120
			y1 = 0
			y2 = 0
			imagenG = cv2.cvtColor(self.imagen0,cv2.COLOR_BGR2GRAY) 					
			imagenT = tip(imagenG)
			_,imagenB = cv2.threshold(imagenT,75,255,cv2.THRESH_BINARY)
			imagenF = cv2.Sobel(imagenB,cv2.CV_8U,1,0, ksize=3)
			if (self.FT<=30):
				print('-----------INCORPORAMIENTO AL CARRIL-----------')
				x1 = 180
				self.FT = self.FT+1
			else: 
				print('-----------SEGUIMIENTO DEL CARRIL-----------')
				x1 = self.x1_h
			x1,y1,x2,y2 = line_detector(imagenF,x1,l,True)
			self.x1_h = x1
			kx = 0.05616094 
			kth = 0.16484354
			self.ex = x1-x_ref
			self.th = np.arctan2(x2-x1,l)
			u = int(round(90-np.arctan(kx*self.ex+kth*self.th)*(180/np.pi))) 
			v = -600
			print('steering ',u)
			print('speed ',v)
			print('************************')
			"""
		 	#Visualizacion
			imagenS = cv2.cvtColor(imagenF,cv2.COLOR_GRAY2BGR)
			imagenS = cv2.circle(imagenS,(x1,y1),3,(0, 0, 255),-1)
			imagenS = cv2.circle(imagenS,(x2,y2),3,(0, 0, 255),-1)
			imagenS = cv2.line(imagenS, (x1,y1), (x2,y2), (0, 0, 255), 2) 
			cv2.imshow('homografia',imagenS)	
			cv2.moveWindow("homografia", 700,30)
			cv2.waitKey(1)
			"""
			self.Vpub.publish(v) 
			self.Spub.publish(u)

		#_______________________________________________________________________________________
		if (value==2):
			print('-----------REBASE-----------')

			if (self.step==0):
				# Conduce por la derecha
				l = 150 #150
				x_ref = 120 
				side = 1
				Nm = 251 # Numero de mediciones
 				# Control de velocidad
				Kv = 250.0
				r_ref = 1.0
				ev = r_ref-r0 
				v = ev*Kv 
				if (v>-100): v = -100
				if (v<-400): v = -400
				# Mide la velocidad del otro coche
				K = 0.1 #1.0/(self.NV_fcar+1.0)
				self.V_fcar = self.V_fcar+K*(v-self.V_fcar)
				self.NV_fcar = self.NV_fcar+1
				if (self.Curve == True):
					self.NV_fcar = 0
				if (self.Curve == False) and (self.NV_fcar==Nm):
					self.FT = 0
					self.step = 1

			if (self.step==1):
				# Conduce por la izquierda
				l = 60 
				x_ref = 80 
				side = -1
				v = 2.0*self.V_fcar
				r_225 = self.R[224]
				if (r_225<1.0): self.step = self.step+1
				"""
				if (abs(x_ref-self.x1_h)<15) and (self.Car==True): 
				print('---------CANCELAR----------')
				l = 150
				x_ref = 120 
				side = 1
				v = self.V_fcar
				"""

			if (self.step==2):
				self.Ev = False
				self.FT = 0
				self.step = 0
				self.V_fcar = 0.0
				self.NV_fcar = 0

			print('v ',v)
			print('V_fcar ', self.V_fcar)
			print('NV_fcar ', self.NV_fcar)
			print('r0 ',r0)

			y1 = 0
			y2 = 0
			imagenG = cv2.cvtColor(self.imagen0,cv2.COLOR_BGR2GRAY) 					
			imagenT = tip(imagenG)
			_,imagenB = cv2.threshold(imagenT,75,255,cv2.THRESH_BINARY)
			imagenF = cv2.Sobel(imagenB,cv2.CV_8U,1,0, ksize=3)
			if (self.FT<=30):
				if (side==1): x1 = 180
				if (side==-1): x1 = 30
				self.FT = self.FT+1
			else: 
				x1 = self.x1_h
			x1,y1,x2,y2 = line_detector(imagenF,x1,l,side) #True// (1)-*der/(-1)--izq 
			self.x1_h = x1
			kx = 0.0487392   
			kth = 0.15340431
			# vrpm	R				Q				Kx						Kth
			# 800		20		0.05*I	0.0487392   0.15340431
			# 800		15		0.05*I	0.05616094	0.16484354 ++
			# 800		10		0.05*I	0.06855525  0.18258912
			self.ex = x1-x_ref
			self.th = np.arctan2(x2-x1,l)
			u = int(round(90-np.arctan(kx*self.ex+kth*self.th)*(180/np.pi))) 
			"""
		 	#Visualizacion
			imagenS = cv2.cvtColor(imagenF,cv2.COLOR_GRAY2BGR)
			imagenS = cv2.circle(imagenS,(x1,y1),3,(0, 0, 255),-1)
			imagenS = cv2.circle(imagenS,(x2,y2),3,(0, 0, 255),-1)
			imagenS = cv2.line(imagenS, (x1,y1), (x2,y2), (0, 0, 255), 2) 
			cv2.imshow('homografia',imagenS)	
			cv2.moveWindow("homografia", 700,30)
			cv2.waitKey(1)
			self.Vpub.publish(v) 
			self.Spub.publish(u) 
			"""
			self.Vpub.publish(v) 
			self.Spub.publish(u)

		
		# Guarda el dataset
		# Lidar
		path = '/home/faber/EK_AutoNOMOS_Sim/src/autonomos_gazebo_simulation/scripts/dataTRAIN/'
		f = open(path+'lidar.csv','a+')
		for r in self.R: f.write('%5.2f	' % (r))
		f.write('\n')
		f.close()
		# Camara
		cv2.imwrite(path+'dataset_passing/'+str(self.i)+'_camara.png', self.imagen0)
		# Entradas de control
		g = open(path+'outputs.csv','a+')
		g.write('%5.2f	%i\n' % (u, v))
		g.close()
		self.i = self.i+1
		

#**********************************************************************************************************************************
#**********************************************************************************************************************************
#**********************************************************************************************************************************
if __name__ == '__main__':
	print("Nodo inicializado: modular_controller_02.py")			
	rospy.init_node('Modular_Controller',anonymous=True)	
	sensors_processing()
	rospy.spin()









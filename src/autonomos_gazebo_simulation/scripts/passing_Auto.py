#! /usr/bin/env python3
from operator import concat
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan
import tensorflow as tf
import matplotlib.pyplot as plt

u = 90
v = -500

class driver():
	def __init__(self):
		self.model = tf.keras.models.load_model('models/RevaseV0.h5', compile = False)

		#V. Lidar
		self.step = 0
		self.R = np.zeros(360)

		#cam
		self.imagen0 = np.zeros((480,640))

		rospy.Subscriber("/app/camera/rgb/image_raw",Image,self.callback_Vis)
		rospy.Subscriber('/scan', LaserScan, self.callback_Lidar)

		self.Vpub = rospy.Publisher('/AutoNOMOS_mini/manual_control/speed',Int16,queue_size=15)				 
		self.Spub = rospy.Publisher('/AutoNOMOS_mini/manual_control/steering',Int16,queue_size=15)
		#rospy.Subscriber("/app/camera/rgb/image_raw",Image,callback_V)

	def callback_Lidar(self, data_lidar):
		i = 0 
		for r in data_lidar.ranges:
			if (r>=3.0): r = 3.0 
			self.R[i] = r
			i = i+1

	def callback_Vis(self, data_vis):
		#self.imagen0 = bridge.imgmsg_to_cv2(data_vis, "bgr8") 	

		im = np.frombuffer(data_vis.data, dtype=np.uint8,).reshape(data_vis.height, data_vis.width, -1)
		im = im[:,:,::-1]
		im = im[220:370, :, :]
		im = cv2.resize(im, (360,66))
		im = (im/255)
		im = tf.cast(im, tf.float32)
		#print(im.shape)

		lidar = self.R[::-1]
		lidar = tf.roll(lidar, 180, axis=0)
		lidar /= 3
		lidarImg = [lidar, lidar, lidar, lidar]
		lidarImg = [lidarImg, lidarImg, lidarImg]
		lidarImg = tf.cast(lidarImg, tf.float32)
		lidarImg = tf.transpose(lidarImg, [1,2,0])
		#print(lidarImg.shape)
		
		self.imagen0 = tf.concat([im, lidarImg], 0)
		#self.imagen0 = tf.cast(self.imagen0, tf.float32)
		self.imagen0 = np.expand_dims(self.imagen0, axis = 0)

		#print(self.imagen0.shape)
		y = self.model(self.imagen0, training = False)
		ang = y.numpy()[0][0]
		vel = y.numpy()[0][1]

		speed = np.abs(vel)*-800*10
		angle = ((ang + 1)/2)*180
		
		self.Vpub.publish(int(speed))
		self.Spub.publish(int(angle))
		print(speed, angle)
		

		
def callback_V(data0):
	global u, v
	#global uArray
	global interpreter, input_details, output_details
	
	im = np.frombuffer(data0.data, dtype=np.uint8,).reshape(data0.height, data0.width, -1)
	im = im[:,:,::-1]
	#imagen0 = bridge.imgmsg_to_cv2(data0, "bgr8")
	#imagen0 = cv2.cvtColor(imagen0, cv2.COLOR_BGR2RGB)
	#im = imagen0.astype(np.float32)
	im = im[270:405, :, :]
	im = cv2.resize(im, (200,66))
	cv2.imshow("img", im)
	cv2.waitKey(1)
	#im = im.astype('float32')
	#im = (im/127.5) - 1
	im = (im/255)
	im = tf.image.adjust_contrast(im, 2)
	im = tf.cast(im, tf.float32)
	
	#im = tf.image.rgb_to_yuv(im)
	#im = cv2.cvtColor(im, cv2.COLOR_BGR2YUV)
	
	im = np.expand_dims(im, axis = 0)	
	#print(im.shape)
	
	#print(im)
	#im = tf.cast(im, tf.float32)
	#im = tf.expand_dims(im, axis = 0)
	#print()
	#interpreter.set_tensor(input_details[0]['index'], im)
	#interpreter.invoke()
	#y = model(im, training = True)[0][0]
	#u = ((y + 1)/2)*180
	#uArray.append(u)
	#u = y
	#u = int(u)
	#print('steering ',u)

	#Vpub.publish(v) 
	#Spub.publish(u)
	#rospy.Rate(10).sleep()


if __name__ == '__main__':
	rospy.init_node('TMR_01',anonymous=True)												
	driver()
	rospy.spin()
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
		self.imagen0 = np.zeros((66,200))

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
		im = im[270:405, :, :]
		im = cv2.resize(im, (200,66))
		cv2.imshow("img", im)
		im = (im/255)
		im = tf.cast(im, tf.float32)
		#print(im.shape)

		lidar = self.R[::-1]
		lidar = tf.roll(lidar, 180, axis=0)
		lidar /= 3
		lidarImg = [
			lidar,
			lidar,
			lidar,
			lidar,
			lidar,
			lidar,
			lidar,
			lidar,
			lidar,
			lidar,
			lidar,
			lidar,
			lidar,
			lidar,
			lidar,
			lidar,
			lidar,
			lidar,
			lidar,
			lidar,
		]
		lidarImg = [lidarImg, lidarImg, lidarImg]
		lidarImg = tf.cast(lidarImg, tf.float32)
		lidarImg = tf.transpose(lidarImg, [1,2,0])

		cv2.imshow("lidar", lidarImg.numpy())
		cv2.waitKey(1)
		#print(lidarImg.shape)

		im = tf.expand_dims(im, axis=0)
		lidarImg = tf.expand_dims(lidarImg, axis=0)
		
		#self.imagen0 = tf.concat([im, lidarImg], 0)
		#self.imagen0 = tf.cast(self.imagen0, tf.float32)
		#imagenLidar = np.expand_dims(lidarImg, axis = 0)
		#self.imagen0 = np.expand_dims(self.imagen0, axis = 0)

		inputs = (im, lidarImg)
		#inputs = tf.expand_dims(inputs, axis=0)
		#print(self.imagen0.shape)
		y = self.model(inputs, training = False)
		ang = y.numpy()[0][0]
		vel = y.numpy()[0][1]

		speed = np.abs(vel)*-800
		angle = ((ang + 1)/2)*180

		if abs(speed) <= 100: speed = -300
		
		self.Vpub.publish(int(speed))
		self.Spub.publish(int(angle))
		print(speed, angle)
		

if __name__ == '__main__':
	rospy.init_node('TMR_01',anonymous=True)												
	driver()
	rospy.spin()
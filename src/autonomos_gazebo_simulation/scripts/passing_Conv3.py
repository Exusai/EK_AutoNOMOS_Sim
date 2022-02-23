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
import collections

u = 90
v = -500

class driver():
	def __init__(self):
		self.model = tf.keras.models.load_model('models/RevaseC3V0.h5', compile = False)

		#V. Lidar
		self.step = 0
		self.R = np.zeros(360)

		#cam
		self.imagen0 = tf.cast(np.ones((180,360,3)), tf.float32)
		self.lidar0 = tf.cast(np.ones((18,360,3)), tf.float32)

		#tempo
		self.video = collections.deque([self.imagen0*10], maxlen=10)
		self.radar = collections.deque([self.lidar0*10], maxlen=10)
		self.processing = False
		

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
		im = np.frombuffer(data_vis.data, dtype=np.uint8,).reshape(data_vis.height, data_vis.width, -1)
		im = im[:,:,::-1]
		im = im[220:405, :, :]
		im = cv2.resize(im, (360,180))
		cv2.imshow("img", im)
		cv2.waitKey(1)
		im = (im/255)
		im = tf.cast(im, tf.float32)
		#print(im.shape)

		lidar = self.R[::-1]
		lidar = tf.roll(lidar, 180, axis=0)
		lidar /= 3
		lidarImg = [lidar, lidar, lidar, lidar,lidar, lidar, lidar, lidar,lidar, lidar, lidar, lidar,lidar, lidar, lidar, lidar,lidar, lidar]
		lidarImg = [lidarImg, lidarImg, lidarImg]
		lidarImg = tf.cast(lidarImg, tf.float32)
		lidarImg = tf.transpose(lidarImg, [1,2,0])
		#print(lidarImg.shape)

		self.video.appendleft(im)
		#self.video.pop()
		self.radar.appendleft(lidarImg)
		#self.radar.pop()
		
		if len(self.video) == 10 and not self.processing:
			tensorVideo = tf.stack(self.video)
			tensorVideo = tf.expand_dims(tensorVideo, axis=0)
			#print(tensorVideo.shape)
			tensorRadar = np.concatenate(self.radar, axis = 0)
			tensorRadar = tf.expand_dims(tensorRadar, axis=0)
			#print(tensorRadar.shape)
			tensorRadar = tf.expand_dims(tensorRadar, axis=0)
			#inputs = tf.expand_dims([tensorVideo, tensorRadar], axis=0)
			inputs = (tensorVideo, tensorRadar)
			pred = self.model(inputs, training = False)

			ang = pred.numpy()[0][0]
			vel = pred.numpy()[0][1]

			speed = np.abs(vel)*-800*100
			angle = ((ang + 1)/2)*180
			
			self.Vpub.publish(int(speed))
			self.Spub.publish(int(angle))
			print(speed, angle)
		else: pass


if __name__ == '__main__':
	rospy.init_node('TMR_01',anonymous=True)												
	#r = rospy.Rate(1)
	driver()
	#r.sleep()
	rospy.spin()
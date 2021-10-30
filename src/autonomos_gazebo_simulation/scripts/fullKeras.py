#! /usr/bin/env python3
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
import tensorflow as tf

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

model = tf.keras.models.load_model('models/model2RGB.h5', compile = False)

def callback_V(data0):
	global u, v
	global FT
	global x1, x2, x1_h, x2_h
	global k
	global interpreter, input_details, output_details
	
	im = np.frombuffer(data0.data, dtype=np.uint8,).reshape(data0.height, data0.width, -1)
	#imagen0 = bridge.imgmsg_to_cv2(data0, "bgr8")
	#imagen0 = cv2.cvtColor(imagen0, cv2.COLOR_BGR2RGB)
	#im = imagen0.astype(np.float32)
	im = im.astype('float32')
	im = im[270:405, :, :]
	#im = tf.image.rgb_to_yuv(im)
	#im = cv2.cvtColor(im, cv2.COLOR_BGR2YUV)
	im = cv2.resize(im, (200,66), interpolation = cv2.INTER_AREA)
	cv2.imshow("img", im)
	cv2.waitKey(1)
	im = np.expand_dims(im, axis = 0)	
	#print(im.shape)
	im = (im/127.5) - 1
	#print(im)
	#im = tf.cast(im, tf.float32)
	#im = tf.expand_dims(im, axis = 0)
	#print()
	#interpreter.set_tensor(input_details[0]['index'], im)
	#interpreter.invoke()
	y = model(im, training = False)[0][0]
	u = ((y + 1)/2)*180
	u = int(u)
	print('steering ',u)

	Vpub.publish(v) 
	Spub.publish(u)


if __name__ == '__main__':
	print("Equipo: DotMEX-CAR")
	print("Nodo inicializado: TMR_01.py")
	rospy.init_node('TMR_01',anonymous=True)												
	Vpub = rospy.Publisher('/AutoNOMOS_mini/manual_control/speed',Int16,queue_size=3)				 
	Spub = rospy.Publisher('/AutoNOMOS_mini/manual_control/steering',Int16,queue_size=3)
	rospy.Subscriber("/app/camera/rgb/image_raw",Image,callback_V)	 						
	rospy.spin()
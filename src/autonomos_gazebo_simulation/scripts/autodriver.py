#! /usr/bin/env python3
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
import tensorflow as tf

u = 90
v = -400 


interpreter = tf.lite.Interpreter('models/modelBinned.tflite')
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
print('Input details: ', input_details)
output_details = interpreter.get_output_details()

def callback_V(data0):
	global u, v
	global interpreter, input_details, output_details
	
	im = np.frombuffer(data0.data, dtype=np.uint8,).reshape(data0.height, data0.width, -1)
	# im = cv2.cvtColor(im, cv2.COLOR_BGR2YUV)
	im = im[270:405, :, :]
	im = cv2.resize(im, (200,66))
	cv2.imshow("img", im)
	cv2.waitKey(1)
	#im = im.astype('float32')
	im = (im/127.5) - 1
	im = tf.cast(im, tf.float32)
	
	#im = tf.image.rgb_to_yuv(im)
	
	im = np.expand_dims(im, axis = 0)	
	#print(im.shape)
	#print(im)
	#im = tf.cast(im, tf.float32)
	#im = tf.expand_dims(im, axis = 0)
	#print()
	interpreter.set_tensor(input_details[0]['index'], im)
	interpreter.invoke()
	y = interpreter.get_tensor(output_details[0]['index'])[0][0]
	#u = ((y + 1)/2)*180
	u = y
	#u = int16(u)
	print('steering ',u)

	Vpub.publish(v) 
	Spub.publish(int(u))

	#rospy.Rate(6).sleep()


if __name__ == '__main__':
	print("Equipo: DotMEX-CAR")
	print("Nodo inicializado: TMR_01.py")
	rospy.init_node('TMR_01',anonymous=True)												
	Vpub = rospy.Publisher('/AutoNOMOS_mini/manual_control/speed',Int16,queue_size=3)				 
	Spub = rospy.Publisher('/AutoNOMOS_mini/manual_control/steering',Int16,queue_size=3)
	rospy.Subscriber("/app/camera/rgb/image_raw",Image,callback_V)	 						
	rospy.spin()
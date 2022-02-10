#! /usr/bin/env python3

#init joy: rosrun joy joy_node 
import cv2
from numpy.core.records import record
import rospy
import numpy as np
from sensor_msgs.msg import Image, Joy
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan#, Imu

joy = 0
trigger = 0

k = 1
record = False
recordBuffer = False

R = np.zeros(360)


def joyReceiver(data):
	global joy, trigger, record, recordBuffer
	joy = data.axes[0]
	trigger = min(data.axes[5], 0)
	a = data.buttons[0]
	if a == 1:
		record = not recordBuffer
		recordBuffer = record
		#print("RECORD OFF")

def clamp(n):
    if n > 180:
        return 180
    elif n < 0:
        return 0
    else:
        return n

def callback_Lidar(data_lidar):
	global R
	i = 0 
	for r in data_lidar.ranges:
		if (r>=3.0): r = 3.0 
		R[i] = r
		i = i+1

def callback_V(data0):
	global trigger, joy
	global k
	global record
	global R

	im = np.frombuffer(data0.data, dtype=np.uint8,).reshape(data0.height, data0.width, -1)
	im = im[:,:,::-1]
	cv2.imshow("img", im)
	cv2.waitKey(1)
	#print('joy ', joy)
	u = int(clamp((joy + 1.0)/2*180))
	#print('steering ', u)
	v = int(trigger * 800)

	if record:
		#f1 = open('steering.csv','a+')
		#f1.write("%5.8f\n" %(joy))
		#f1.close()
		#cv2.imwrite('dataset/im_road'+str(k)+'.png',im)
		
		path = '/home/faber/EK_AutoNOMOS_Sim/src/autonomos_gazebo_simulation/scripts/dataTRAIN/'
		f = open(path+'lidar.csv','a+')
		for r in R: f.write('%5.2f	' % (r))
		f.write('\n')
		f.close()
		# Camara
		cv2.imwrite(path+'dataset_passing/'+str(k)+'_camara.png', im)
		# Entradas de control
		g = open(path+'outputs.csv','a+')
		g.write('%5.2f	%i\n' % (u, v))
		g.close()
		k = k+1

		print(" Recording    ", end="\r")
	else:
		print(" NOT Recording", end="\r")

	Vpub.publish(v) 
	Spub.publish(u)


if __name__ == '__main__':
	print("Iniciando")
	print("Gatillo para acelerar")
	print("A para grabar")
	print("--------------")
	rospy.init_node('TMR_01', anonymous=True)
	Vpub = rospy.Publisher('/AutoNOMOS_mini/manual_control/speed',Int16,queue_size=3)
	Spub = rospy.Publisher('/AutoNOMOS_mini/manual_control/steering',Int16,queue_size=3)
	rospy.Subscriber("/app/camera/rgb/image_raw",Image,callback_V)	
	rospy.Subscriber("/joy", Joy, joyReceiver)
	rospy.Subscriber('/scan', LaserScan, callback_Lidar)
	rospy.spin()
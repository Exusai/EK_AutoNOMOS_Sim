#! /usr/bin/env python3

#init joy: rosrun joy joy_node 
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image, Joy
from std_msgs.msg import Int16

joy = 0
trigger = 0
def joyReceiver(data):
	global joy, trigger
	joy = data.axes[0]
	trigger = min(data.axes[5], 0)
	#print(u)

def clamp(n):
    if n > 180:
        return 180
    elif n < 0:
        return 0
    else:
        return n

def callback_V(data0):
	global trigger, joy

	im = np.frombuffer(data0.data, dtype=np.uint8,).reshape(data0.height, data0.width, -1)
	cv2.imshow("img", im)
	cv2.waitKey(1)
	#print('joy ', joy)
	u = int(clamp((joy + 1.0)/2*180))
	#print('steering ', u)
	v = int(trigger * 800)

	Vpub.publish(v) 
	Spub.publish(u)


if __name__ == '__main__':
    print("Iniciando")
    rospy.init_node('TMR_01', anonymous=True)
    Vpub = rospy.Publisher('/AutoNOMOS_mini/manual_control/speed',Int16,queue_size=3)
    Spub = rospy.Publisher('/AutoNOMOS_mini/manual_control/steering',Int16,queue_size=3)
    rospy.Subscriber("/app/camera/rgb/image_raw",Image,callback_V)	
    rospy.Subscriber("/joy", Joy, joyReceiver)
    rospy.spin()
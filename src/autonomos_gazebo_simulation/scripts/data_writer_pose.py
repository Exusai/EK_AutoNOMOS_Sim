#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D

#************************************************************************************************************
#************************************************************************************************************
#************************************************************************************************************
def callback_O(data0):
	t = rospy.get_time()
	x = data0.x
	y = data0.y
	yaw = data0.theta
	f = open('/home/faber/EK_AutoNOMOS_Sim/src/autonomos_gazebo_simulation/scripts/eval/pose_aanual_on_mov.csv','a+')
	f.write("%5.2f	%5.2f	%5.2f	%5.2f\n" % (x, y, yaw, t))
	f.close()
#************************************************************************************************************
#************************************************************************************************************
#************************************************************************************************************
def callback_1(data1):
	t = rospy.get_time()
	x = data1.x
	y = data1.y
	yaw = data1.theta
	f = open('/home/faber/EK_AutoNOMOS_Sim/src/autonomos_gazebo_simulation/scripts/eval/pose_dummycar1_01.csv','a+')
	f.write("%5.2f	%5.2f	%5.2f	%5.2f\n" % (x, y, yaw, t))
	f.close()
#************************************************************************************************************
#************************************************************************************************************
#************************************************************************************************************
if __name__ == '__main__':
	print "*************************************"
	print "	...Guardando datos..."
	print "*************************************"
	rospy.init_node('data_write', anonymous=True)	# Nombre del nodo
	rospy.Subscriber('/AutoNOMOS_mini/real_pose_from_gazebo',Pose2D, callback_O)
	#rospy.Subscriber('/Dyn_car01/real_pose_from_gazebo',Pose2D, callback_1)
	rospy.spin()		


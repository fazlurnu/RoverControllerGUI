#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu

def callback(data):
	x=data.linear_acceleration.x
	y=data.linear_acceleration.y
	z=data.linear_acceleration.z
	#rospy.loginfo(rospy.get_caller_id() + "\nlinearAcc:\n" + str(x))
	rospy.loginfo("\nlinearAcc:\n" + "x: " + str(x) + 
		"\ny: " + str(y) + "\nz: " + str(z))

def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("/mavros/imu/data", Imu, callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterrupException:
		pass

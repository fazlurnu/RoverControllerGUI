#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from mavros_msgs.msg import RCOut

def callback(data):
	z=data.linear_acceleration.z
	rospy.loginfo("\nlinearAcc:\n" + "z: " + str(z))

def callback_rcout(data):
	channels=data.channels
	rospy.loginfo("\nPWM Out:" + str(channels))

def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("/mavros/imu/data", Imu, callback)
	rospy.Subscriber("/mavros/rc/out", RCOut, callback_rcout)
	rospy.spin()

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterrupException:
		pass

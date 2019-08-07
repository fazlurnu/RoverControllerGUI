#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_msgs.msg import String
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL

import tf
import math
import time

exec_time = 1/25;

def setMode():
	print("Set mode")
	rospy.wait_for_service('/mavros/set_mode')
	try:
		modeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
		modeResponse = modeService(0, 'MANUAL')
		rospy.loginfo("\nMode Response: " + str(modeResponse))
		print("Mode set")
	except rospy.ServiceException as e:
		print("Service call failed: %s" %e)

def arm():
	print("Set arming")
	rospy.wait_for_service('/mavros/cmd/arming')
	try:
		armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		armResponse = armService(True)
		rospy.loginfo(armResponse)
		print("Armed")
	except rospy.ServiceException as e:
		print("Service call failed: %s" %e)

def disarm():
	print("Set disarming")
	rospy.wait_for_service('/mavros/cmd/arming')
	try:
		armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		armResponse = armService(False)
		rospy.loginfo(armResponse)
		print("Disarmed")
	except rospy.ServiceException as e:
		print("Service call failed: %s" %e)

class Control():
	def __init__(self):
		self.pitch = 0
		self.error = 0
		self.pwm2 = 0
		self.msg = [1500, 1500, 1100, 1500, 1000, 1500, 1500, 1500]
		rospy.init_node('control', anonymous=True)
		self.pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)
		rospy.Subscriber("/mavros/imu/data", Imu, self.getPitch)
		rospy.spin()

	def getPitch(self, msg):
		#rospy.loginfo("lis")
		quaternion = (
		msg.orientation.x,
		msg.orientation.y,
		msg.orientation.z,
		msg.orientation.w)

		euler = tf.transformations.euler_from_quaternion(quaternion)
		#roll = euler[0] * 180 / math.pi
		self.pitch = euler[1] * 180 / math.pi
		#yaw = euler[2] * 180 / math.pi
		#pitchRate = data.angular_velocity.y
		refPitchAngle = 10;
		#refPitchRate = 0;
		self.error = refPitchAngle - self.pitch;
		#pitchRateError = refPitchRate - pitchRate;
		pwm2 = self. error*10+1500;
		if(pwm2>1700):
			self.pwm2 = 1700
		elif (pwm2<1300):
			self.pwm2 = 1300
		#rospy.loginfo(self.pwm2)
		self.command()

	def command(self):
		rospy.loginfo("com")
	    	rate = rospy.Rate(20) # 10hz
		flag = True
		while not rospy.is_shutdown() and flag:
			flag=False
			self.msg = [self.pwm2, self.pwm2, 1500, 1500, 1000, 1500, 1500, 1500]
			self.pub.publish(self.msg)
			rospy.loginfo(self.msg)
			#begin=time.time()
			rate.sleep()

if __name__ == '__main__':
	#try:
		print("Program starts")
		#setMode()
		#arm()
		#time.sleep(3)
		#control()
		myCon = Control()
		#myCon.getPitch()
		myCon.command()
	#except rospy.ROSInterrupException:
	#	pass

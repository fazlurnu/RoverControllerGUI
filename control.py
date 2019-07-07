#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL

import tf
import math
import time

exec_time = 1/25;

def callback(data):
	pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)
    	rate = rospy.Rate(100) # 10hz
	begin = rospy.Time()
	flag = True

	while not rospy.is_shutdown() and flag:
		flag=False
		quaternion = (
		data.orientation.x,
		data.orientation.y,
		data.orientation.z,
		data.orientation.w)
	
		euler = tf.transformations.euler_from_quaternion(quaternion)
		roll = euler[0] * 180 / math.pi
		pitch = euler[1] * 180 / math.pi
		yaw = euler[2] * 180 / math.pi
		#pitchRate = data.angular_velocity.y
		refPitchAngle = 10;
		#refPitchRate = 0;
		error = refPitchAngle - pitch;
		#pitchRateError = refPitchRate - pitchRate;
		pwm2 = error*10+1500;
		if(pwm2>1700):
			pwm2 = 1700
		elif (pwm2<1300):
			pwm2 = 1300
		list = [pwm2, pwm2, 1500, 1500, 1000, 1500, 1500, 1500]
		pub.publish(list)
		#rospy.loginfo(error)
		rate.sleep()

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

def control():
	rospy.init_node('control', anonymous=True)
	rospy.Subscriber("/mavros/imu/data", Imu, callback)
	rospy.spin()

if __name__ == '__main__':
	#try:
		print("Program starts")
		setMode()
		arm()
		time.sleep(3)
		control()
	#except rospy.ROSInterrupException:
	#	pass

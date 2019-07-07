#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt64

def talker():
    pub = rospy.Publisher('chatter', UInt64, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    angle = 10;
    increment = 5;
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()	
	angle+=increment;
       	rospy.loginfo(angle)
        pub.publish(angle)
        rate.sleep()
	if(angle>120 or angle<10):
		increment*=-1;

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

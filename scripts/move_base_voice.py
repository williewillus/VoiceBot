#!/usr/bin/env python
from __future__ import division
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist

#based on voice_cmd_vel
class move_base_voice:
    def __init__(self):
	rospy.on_shutdown(self.cleanup)
	self.msg = Twist()
	
	self.pub_ = rospy.Publisher('cmd_vel', Twist)
	rospy.Subscriber('recognizer/output', String, self.speechCb)
	
	r = rospy.Rate(10.0)
	
	while not rospy.is_shutdown():
	    self.pub_.publish(self.msg)
	    r.sleep()

    def speechCb(self, msg):
	#rospy.loginfo(msg.data)
	#commands go here
	if msg.data.find("turn") > -1:
	    if msg.data.find("ninety degrees") > -1:
	    	self.msg.angular.z = self.msg.angular.z + math.radians(90)

	self.pub_.publish(self.msg)
    def cleanup(self):
	twist = Twist()
	self.pub_.publish(twist)

if __name__ == '__main__':
    rospy.init_node('move_base_voice')
    try:
        move_base_voice()
    except rospy.ROSInterruptException:
        pass

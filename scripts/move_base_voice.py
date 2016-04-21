#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import roslib
roslib.load_manifest('move_base')
import actionlib
import tf

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

goal = MoveBaseGoal()

#based on voice_cmd_vel
class move_base_voice:
    def __init__(self):
	rospy.on_shutdown(self.cleanup)

	rospy.Subscriber('recognizer/output', String, self.speechCb)

    def speechCb(self, msg):
	#rospy.loginfo(msg.data)
	#commands go here
	if msg.data.find("turn") > -1:
	    if msg.data.find("ninety degrees") > -1:
		q = tf.transformations.quaternion_from_euler(0, 0, math.radians(90))
		
	    	goal.target_pose.pose.orientation.x = q[0]
		goal.target_pose.pose.orientation.y = q[1]
		goal.target_pose.pose.orientation.z = q[2]
		goal.target_pose.pose.orientation.w = q[3]
	
    def cleanup(self):
	goal = MoveBaseGoal()

if __name__ == '__main__':
    rospy.init_node('move_base_voice')
    try:
        move_base_voice()
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	#client.wait_for_server()
	while not rospy.is_shutdown():
	    client.send_goal(goal)
	    client.wait_for_result(rospy.Duration.from_sec(5.0))
    except rospy.ROSInterruptException:
        pass

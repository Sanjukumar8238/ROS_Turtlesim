#!/usr/bin/env python

import rospy
from beginner_tutorials.srv import *

def move_square_client():
	rospy.wait_for_service('move_pentagon')
	try:
		move_pentagon = rospy.ServiceProxy('move_pentagon', MovePentagon)
		move_pentagon(3.0,1)

	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

if __name__ == "__main__":
	move_square_client()

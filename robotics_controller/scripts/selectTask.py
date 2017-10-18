#!/usr/bin/env python
import roslib
roslib.load_manifest('robotics_controller')

import rospy
import actionlib
import sys, os, fcntl, time
import termios, tty, select

from std_msgs.msg import Float64
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal

from robotics_controller.msg import Trajectory
from robotics_controller.msg import Task
from robotics_controller.srv import move_armResponse
from robotics_controller.srv import move_arm

def isData():
	
	return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def main():


	print "Waiting for tasks to load up...."
	rospy.wait_for_service("move_arm")
			
	print "The wait is over. All tasks have been loaded"

	moveArm = rospy.ServiceProxy("move_arm", move_arm)

	print "You can now enter which task you want to enter" 

	print "If you want to switch to another tasl, press Ctr+C"
	while True:
		
		task = input("Which task do you want the robot to execute: ")
		
		try: 
			
			while True:
			
				resp = moveArm(task)

		except KeyboardInterrupt:
			
			print "You can now change tasks"
			
			

if __name__ == "__main__":

	main()

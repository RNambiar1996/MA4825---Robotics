#!/usr/bin/env python
import roslib
roslib.load_manifest('robotics_controller')

import rospy
import actionlib
import sys, os, fcntl, time
import termios

from std_msgs.msg import Float64
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal

from robotics_controller.msg import Trajectory
from robotics_controller.msg import Task
from robotics_controller.srv import move_armResponse


def isData():
	
	return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def main():

	rospy.wait_for_service("move_arm")
			
	moveArm = rospy.ServiceProxy("move_arm", move_arm)

	while True:
		
		task = input("Which task do you want the robot to execute")

		print "Enter ESC to stop the arm from executing the task"
		
		old_settings = termios.tcgetattr(sys.stdin)

		try:
		
			tty.setcbreak(sys.stdin.fileno())
			
			while True:

				resp = moveArm(task)

				if isData():

					c = sys.stdin.read(1)

					if c == '\x1b':

						break
					


if __name__ == "__main__":

	main()

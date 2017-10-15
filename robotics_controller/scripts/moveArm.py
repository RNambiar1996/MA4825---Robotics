#!/usr/bin/env python
import roslib
roslib.load_manifest('robotics_controller')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from robotics_controller.msg import Trajectory
from robotics_controller.msg import Task

from robotics_controller.srv import move_armResponse
from robotics_controller.srv import move_arm


		          

def moveArm(req):
	
	for x in range(0,3):

		joint_1 = req.goal_points.joint_values[x].joint_1
		joint_2 = req.goal_points.joint_values[x].joint_2
		joint_3 = req.goal_points.joint_values[x].joint_3

		rospy.logerr("joint1: " +str(joint_1) +". joint2: " +str(joint_2) + " joint3: " +str(joint_3))
		rospy.logerr("sending goal")

		arm.move_joint([joint_1, joint_2, joint_3])	

	resp = move_armResponse()

	resp.success = True
	
	return resp	



def main():

	rospy.Service("move_arm", move_arm, moveArm)

	rospy.spin()




if __name__ == "__main__":

	rospy.init_node("moveArm")

	arm = Joint("f_arm")

	main()

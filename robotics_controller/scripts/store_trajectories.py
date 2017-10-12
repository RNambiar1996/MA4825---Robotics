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
from robotics_controller.msg import Potentiometer
from robotics_controller.msg import Trajectory

from robotics_controller.srv import move_armResponse
from robotics_controller.srv import move_arm


class Joint:

	def __init__(self, motor_name):
		#arm_name should be b_arm or f_arm
		self.name = motor_name           
		self.jta = actionlib.SimpleActionClient('/'+self.name+'/follow_joint_trajectory', FollowJointTrajectoryAction)
		rospy.loginfo('Waiting for joint trajectory action')
		self.jta.wait_for_server()
		rospy.loginfo('Found joint trajectory action!')

	def move_joint(self, angles):
		goal = FollowJointTrajectoryGoal()                  
		char = self.name[0] #either 'f' or 'b'
		goal.trajectory.joint_names = ['joint_1'+char, 'joint_2'+char, 'joint_3'+char]
		point = JointTrajectoryPoint()
		point.positions = angles
		point.time_from_start = rospy.Duration(0.05)                   
		goal.trajectory.points.append(point)
		self.jta.send_goal_and_wait(goal)
		          

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

	rospy.init_node("move_arm")

	arm = Joint("f_arm")

	main()

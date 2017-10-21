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

class Joint:

	def __init__(self, motor_name):
		#arm_name should be b_arm or f_arm
		self.name = motor_name           
		self.jta = actionlib.SimpleActionClient('/'+self.name+'/follow_joint_trajectory', FollowJointTrajectoryAction)
		#rospy.loginfo('Waiting for joint trajectory action')
		self.jta.wait_for_server()
		#rospy.loginfo('Found joint trajectory action!')

	def move_joint(self, angles):
		goal = FollowJointTrajectoryGoal()                  
		char = self.name[0] #either 'f' or 'b'
		goal.trajectory.joint_names = ['joint_1'+char, 'joint_2'+char, 'joint_3'+char]
		point = JointTrajectoryPoint()
		point.positions = angles
		point.velocities = [0.5,0.5,0.5]
		point.time_from_start = rospy.Duration(0.05)                   
		goal.trajectory.points.append(point)
		self.jta.send_goal_and_wait(goal)


def main():

	j1_pub = rospy.Publisher("/joint1_controller/command", Float64, queue_size=1)
		
	j2_pub = rospy.Publisher("/joint1_controller/command", Float64, queue_size=1)

	j3_pub = rospy.Publisher("/joint1_controller/command", Float64, queue_size=1)

	gripper_pub = rospy.Publisher("/joint1_controller/command", Float64, queue_size=1)

	init = Float64()
	
	init.data = 0.0
	
	time.sleep(2)
	print "init j1"

	j1_pub.publish(init)
	
	print "init j2"
	j2_pub.publish(init)

	init.data = 2.5
	print "init j3"
	j3_pub.publish(init)

	print "init j4"
	init.data = 1.4
	gripper_pub.publish(init)

	rospy.spin()
	
if __name__ == "__main__":

	rospy.init_node('joint_pose_tester')
	main()




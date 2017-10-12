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
from robotics_controller.srv import move_arm


def move_arm_callback(joint_values):
	
	global trajectories
	
	joint1 = joint_values.joint_1
	joint2 = joint_values.joint_2
	joint3 = joint_values.joint_3

	joint1 = joint1/960*3.14
	joint2 = joint2/960*3.14
	joint3 = joint3/960*3.14

	rospy.logerr("Joint1: " +str(joint1))
	rospy.logerr("Joint2: " +str(joint2))
	rospy.logerr("Joint3: " +str(joint3)) 

	rospy.logerr("Status" +str(status))
	
	button_pressed = joint_values.button_pressed

	pub_1 = rospy.Publisher("joint1_controller/command", Float64, queue_size=10)
	pub_2 = rospy.Publisher("joint2_controller/command", Float64, queue_size=10)
	pub_3 = rospy.Publisher("joint3_controller/command", Float64, queue_size=10)


	if button_pressed == 0:

		joint_value_pub = Float64()

		joint_value_pub.data = joint1
		pub_1.publish(joint_value_pub)

		joint_value_pub.data = joint2
		pub_2.publish(joint_value_pub)

		joint_value_pub.data = joint3
		pub_3.publish(joint_value_pub)

	else:
		joint_values.joint_1 = joint1
		joint_values.joint_2 = joint2
		joint_values.joint_3 = joint3
		trajectories.joint_values.append(joint_values)
	
		size = len(trajectories.joint_values)

		rospy.logerr("Size of trajectory")
		rospy.logerr(size)

		if size == 3:
			rospy.logerr("In here")
			moveArm = rospy.ServiceProxy("move_arm", move_arm)
			resp = moveArm(trajectories)
			trajectories = Trajectory()		
	

rospy.init_node("joint_position_tester")

trajectories = Trajectory()
status = 0
sub = rospy.Subscriber("joint_values", Potentiometer, move_arm_callback)
rospy.wait_for_service("move_arm")	
rospy.spin()


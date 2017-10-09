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
              

def move_arm_callback(joint_value):

	rospy.logerr("In callback")
	
	joint1 = joint_value.joint_1
	joint2 = joint_value.joint_2
	joint3 = joint_value.joint_3

	joint1 = joint1/960*3.14
	joint2 = joint2/960*3.14
	joint3 = joint3/960*3.14

	rospy.logerr("Joint1: " +str(joint1))
	rospy.logerr("Joint2: " +str(joint2))
	rospy.logerr("Joint3: " +str(joint3)) 

	#pub_1 = rospy.Publisher("joint1_controller/command", Float64, queue_size=10)
	#pub_2 = rospy.Publisher("joint2_controller/command", Float64, queue_size=10)
	#pub_3 = rospy.Publisher("joint3_controller/command", Float64, queue_size=10)
		
	#joint_value_pub = Float64()

	#joint_value_pub.data = joint1
	#pub_1.publish(joint_value_pub)

	#joint_value_pub.data = joint2
	#pub_2.publish(joint_value_pub)

	#joint_value_pub.data = joint3
	#pub_3.publish(joint_value_pub)

	arm.move_joint([joint1, joint2, joint3])
	

def main():
      	
	      
	sub = rospy.Subscriber("joint_values", Potentiometer, move_arm_callback);	
	rospy.spin()

                        
if __name__ == '__main__':
	rospy.init_node('joint_position_tester')
	arm = Joint('f_arm')
	main()

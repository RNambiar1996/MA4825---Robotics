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
		point.time_from_start = rospy.Duration(0.05)                   
		goal.trajectory.points.append(point)
		self.jta.send_goal_and_wait(goal)


class TaskSelector():

	def __init__(self, tasks, nTasks):
		self.tasks = tasks
		self.nTasks = nTasks
		self.arm = Joint("f_arm") 
		self.fl = fcntl.fcntl(sys.stdin.fileno(), fcntl.F_GETFL)
		fcntl.fcntl(sys.stdin.fileno(), fcntl.F_SETFL, self.fl | os.O_NONBLOCK)		

		moveArm = rospy.Service("move_arm", move_arm, self.moveArm)

	
	def moveArm(self, req):
		
		choice_task = req.nTask

		task = self.tasks[choice_task-1]
		nTraj = task.nTraj

		for i in range(0, nTraj):

			j1 = task.joint_values[i].joint_1
			j2 = task.joint_values[i].joint_2
			j3 = task.joint_values[i].joint_3

			j1 = (j1/950)*3.14159
			j2 = (j2/950)*3.14159
			j3 = (j3/950)*3.14159

			self.arm.move([j1,j2,j3])	

		resp = move_armResponse()
		resp.success = True

		return resp
	

			
class ServoResponse():

	def __init__(self, nTasks):

		self.flag = False
		self.tasks = []
		self.servo_1 = Float64()
		self.servo_2 = Float64()
		self.servo_3 = Float64()
		self.nTasks = nTasks
		self.taskCounter = 0

		if self.flag == False:
			self.sub_trajectory = rospy.Subscriber("trajectory", Trajectory, self.move_arm_callback)
			self.sub_tasks = rospy.Subscriber("task",Task, self.store_tasks) 

		self.pub_servo_1 = rospy.Publisher("joint1_controller/command", Float64, queue_size=10)
		self.pub_servo_2 = rospy.Publisher("joint2_controller/command", Float64, queue_size=10)
		self.pub_servo_3 = rospy.Publisher("joint3_controller/command", Float64, queue_size=10)

	def move_arm_callback(self, trajectory):
		
		self.servo_1.data = (trajectory.joint_1/938)*3.14159265359
		self.servo_2.data = (trajectory.joint_2/938)*3.14159265359
		self.servo_3.data = (trajectory.joint_3/938)*3.14159265359

		
		if self.flag == False:
			#rospy.logerr("Servo 1 data: " +str(self.servo_1.data))
			#rospy.logerr("Servo 2 data: " +str(self.servo_2.data))
			#rospy.logerr("Servo 3 data: " +str(self.servo_3.data))
			self.pub_servo_1.publish(self.servo_1)
			self.pub_servo_2.publish(self.servo_2)
			self.pub_servo_3.publish(self.servo_3)		
	
	def store_tasks(self, task):
		
		print("Task: ")
		print(task)
		self.tasks.append(task)
		self.taskCounter = self.taskCounter + 1
		print("Task no: " +str(self.taskCounter) +" recorded\n")
		print("You can enter " +str(self.nTasks-self.taskCounter) +" more tasks")
				
		if self.taskCounter == self.nTasks:
		
			print("All tasks stored")
			self.flag = True
			self.sub_trajectory.unregister()
			self.sub_tasks.unregister()

			self.task_selector = TaskSelector(self.tasks, self.nTasks)

			

			
def main():
	
	rospy.init_node("joint_position_tester")

	print("Hello. Welcome to the robot instructor program\n")

	nTasks = input("How many tasks do you want to train the robot for: ")

	response = raw_input("Are you ready to continue? Enter(y/n)")

	if response[0] == 'y':
		servo_response = ServoResponse(nTasks)

	rospy.spin()
			

if __name__ == "__main__":
	main()


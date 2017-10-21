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
from robotics_controller.srv import move_arm
from robotics_controller.srv import move_armResponse

from dynamixel_controllers.srv import SetSpeed
from dynamixel_controllers.srv import SetSpeedResponse



class TaskSelector():

	def __init__(self, tasks, nTasks):
		self.tasks = tasks
		self.nTasks = nTasks
		self.fl = fcntl.fcntl(sys.stdin.fileno(), fcntl.F_GETFL)
		self.pub_servo_1 = rospy.Publisher("/joint1_controller/command", Float64, queue_size=1)
		self.pub_servo_2 = rospy.Publisher("/joint2_controller/command", Float64, queue_size=1)
		self.pub_servo_3 = rospy.Publisher("/joint3_controller/command", Float64, queue_size=1)
		self.pub_gripper = rospy.Publisher("/gripper_controller/command", Float64, queue_size=1)

		self.servo_1 = Float64()
		self.servo_2 = Float64()
		self.servo_3 = Float64()
		self.gripper = Float64()

		
		fcntl.fcntl(sys.stdin.fileno(), fcntl.F_SETFL, self.fl | os.O_NONBLOCK)		

		moveArm = rospy.Service("move_arm", move_arm, self.moveArm)

	
	def moveArm(self, req):
		
		choice_task = req.nTask

		print "There are " +str(len(self.tasks)) + " tasks stored"

		print "Chosen task: " +str(choice_task)

		task = self.tasks[choice_task-1]
		nTraj = task.nTraj
		
		for i in range(0, nTraj):

			j1 = task.joint_values[i].joint_1
			j2 = task.joint_values[i].joint_2
			j3 = task.joint_values[i].joint_3
			g = task.joint_values[i].gripper

			self.servo_1.data = (j1/960)*3.14159

			if (j2 > 20 and j2 <= 34):
				self.servo_2.data = -0.035*j2+2.05
			elif (j2 > 34 and j2 <= 100):
				self.servo_2.data = -0.00605*j2+1.05
			elif (j2 > 100):
				self.servo_2.data = -0.000517*j2+0.502
			
			if self.servo_2.data < 0.0:
				self.servo_2.data = 0.0
			if self.servo_2.data > 1.27:
				self.servo_2.data = 1.345
		
			

			self.servo_3.data = -0.07*j3+2.85
			if self.servo_3.data < 1.1:
				self.servo_3.data = 1.1
			if self.servo_3.data > 2.5:
				self.servo_3.data = 2.5

			self.gripper.data = g

			print "j1: " +str(self.servo_1.data)
			print "j2: " +str(self.servo_2.data)
			print "j3: " +str(self.servo_3.data)

			
			time.sleep(0.01)
			self.pub_servo_1.publish(self.servo_1)
			time.sleep(0.01)
			self.pub_servo_2.publish(self.servo_2)
			time.sleep(0.01)
			self.pub_servo_3.publish(self.servo_3)
			self.pub_gripper.publish(self.gripper)

			time.sleep(2.0)

			

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
		self.gripper = Float64()
		self.nTasks = nTasks
		self.taskCounter = 0
		self.initialised = False

		setSpeed_j1()
		setSpeed_j2()
		setSpeed_j3()
		setSpeed_gripper()

		self.pub_servo_1 = rospy.Publisher("/joint1_controller/command", Float64, queue_size=1)
		self.pub_servo_2 = rospy.Publisher("/joint2_controller/command", Float64, queue_size=1)
		self.pub_servo_3 = rospy.Publisher("/joint3_controller/command", Float64, queue_size=1)
		self.pub_gripper = rospy.Publisher("/gripper_controller/command", Float64, queue_size=1)

		self.initialise()

		self.sub_trajectory = rospy.Subscriber("trajectory", Trajectory, self.move_arm_callback)
		self.sub_tasks = rospy.Subscriber("task",Task, self.store_tasks) 

	
	def initialise(self):

		print "Initialising"
	 	j1_init = Float64()
		j2_init = Float64()
		j3_init = Float64()
		gripper_init = Float64()

		j1_init.data = 0.0
		j2_init.data = 0.0
		j3_init.data = 2.5
		gripper_init.data = 1.4
		
		print("j1 init: "+str(j1_init))
		print("j2 init: " +str(j2_init))

		time.sleep(2)

		self.pub_servo_1.publish(j1_init)
		self.pub_servo_2.publish(j2_init)
		self.pub_servo_3.publish(j3_init)		
		self.pub_gripper.publish(gripper_init)

		time.sleep(5)

		print "Intialisation complete"
		self.initialised = True		

	def move_arm_callback(self, trajectory):
		
		self.servo_1.data = (trajectory.joint_1/938)*3.14159265359


		#mapping pot2 values to servo values so that equal angle increments are seen in both
		if (trajectory.joint_2 > 20 and trajectory.joint_2 <= 34):
			self.servo_2.data = -0.035*trajectory.joint_2+2.05
		elif (trajectory.joint_2 > 34 and trajectory.joint_2 <= 100):
			self.servo_2.data = -0.00605*trajectory.joint_2+1.05
		elif (trajectory.joint_2 > 100):
			self.servo_2.data = -0.000517*trajectory.joint_2+0.502
			
		if self.servo_2.data < 0.0:
			self.servo_2.data = 0.0
		if self.servo_2.data > 1.27:
			self.servo_2.data = 1.345
			

		self.servo_3.data = -0.07*trajectory.joint_3+2.85
		if self.servo_3.data < 1.1:
			self.servo_3.data = 1.1
		if self.servo_3.data > 2.5:
			self.servo_3.data =2.5

		self.gripper.data = trajectory.gripper
		
		if self.flag == False and self.initialised == True:
			#rospy.logerr("Servo 1 data: " +str(self.servo_1.data))
			#rospy.logerr("Servo 2 data: " +str(self.servo_2.data))
			#rospy.logerr("Servo 3 data: " +str(self.servo_3.data))
			self.pub_servo_1.publish(self.servo_1)
			self.pub_servo_2.publish(self.servo_2)
			self.pub_servo_3.publish(self.servo_3)		
			self.pub_gripper.publish(self.gripper)

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

			
#Functions to set speed of arm joints
def setSpeed_j1():

	rospy.wait_for_service("/joint1_controller/set_speed")
	print "Setting j1 speed"

	setSpeed = rospy.ServiceProxy("/joint1_controller/set_speed", SetSpeed)

	resp = setSpeed(1.0)	

def setSpeed_j2():
	
	rospy.wait_for_service("/joint2_controller/set_speed")
	
	print "Setting j2 speed"

	setSpeed = rospy.ServiceProxy("/joint2_controller/set_speed", SetSpeed)

	resp = setSpeed(1.0)

def setSpeed_j3():
		
	rospy.wait_for_service("/joint3_controller/set_speed")

	print "Setting j3 speed"

	setSpeed = rospy.ServiceProxy("/joint3_controller/set_speed", SetSpeed)

	resp = setSpeed(1.0)
	

def setSpeed_gripper():
	
	rospy.wait_for_service("/gripper_controller/set_speed")

	print "Setting gripper speed"

	setSpeed = rospy.ServiceProxy("/gripper_controller/set_speed", SetSpeed)

	resp = setSpeed(1.0)

#intialisation function to set all joints to their initial zero values
			
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


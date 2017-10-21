# Task Instructable Robotic Arm

This ROS package can be used to instruct a robotic arm to learn certain tasks with the aid of a smaller mock arm. 
The mock arm joints are connected to potentiometers whose values are recorded using an attached arduino. The arduino communicated to the main node via rosserial_arduino

The main script which records the joint values and replays them is servo_response.py. It also creates a service which can be used by other nodes to replay certain tasks. 

The arm we used was a 3 dof robotic arm with a gripper, but the code can be extended to any dof arm. 

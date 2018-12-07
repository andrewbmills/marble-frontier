#!/usr/bin/env python
import sys
import numpy as np
import guidance
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from gazebo_msgs.msg import LinkStates

class guidance_controller:
	def getPosition(self, data): # Position subscriber callback function
		self.position = np.array([data.pose[77].position.x, \
			data.pose[77].position.y, data.pose[77].position.z])
		q = np.array([data.pose[77].orientation.x, \
			data.pose[77].orientation.y, data.pose[77].orientation.z, \
			data.pose[77].orientation.w])
		self.R = np.zeros((3,3))
		self.R[0,0] = q[3]*q[3] + q[0]*q[0] - q[1]*q[1] - q[2]*q[2]
		self.R[0,1] = 2.0*(q[0]*q[1] - q[3]*q[2])
		self.R[0,2] = 2.0*(q[3]*q[1] + q[0]*q[2])

		self.R[1,0] = 2.0*(q[0]*q[1] + q[3]*q[2])
		self.R[1,1] = q[3]*q[3] - q[0]*q[0] + q[1]*q[1] - q[2]*q[2]
		self.R[1,2] = 2.0*(q[1]*q[2] - q[3]*q[0])

		self.R[2,0] = 2.0*(q[0]*q[2] - q[3]*q[1])
		self.R[2,1] = 2.0*(q[3]*q[0] + q[1]*q[2])
		self.R[2,2] = q[3]*q[3] - q[0]*q[0] - q[1]*q[1] + q[2]*q[2]

		self.positionUpdated = 1
		return

	def getPath(self, data): # Path subscriber callback function
		self.path = np.empty((3,0))
		for i in range(0,len(data.poses)):
			point = np.array([[data.poses[i].pose.position.x], [data.poses[i].pose.position.y], [data.poses[i].pose.position.z]])
			self.path = np.append(self.path, point, axis=1)
		print('path received of size: %d' % self.path.shape[1])
		self.pathUpdated = 1
		return

	def updateCommand(self): # Updates the twist command for publishing
		# Check if the subscribers have updated the robot position and path
		if (self.path.shape[1] < 1):
			print("No guidance command, path is empty.")
			return

		# Convert the body frame velocity of the robot to inertial frame
		if (self.command.linear.x == 0.0): # Set velocity to nonzero if it is currently zero
			self.command.linear.x = 1.0
		velocity = np.array([[self.command.linear.x], [self.command.linear.y], [self.command.linear.z]]) # using commanded velocity for now (use actual later)
		velocity = np.matmul(self.R, velocity)
		# print('velocity = ', velocity)

		# Find the lookahead/carrot point for the guidance controller
		p_L2, v_L2 = guidance.find_Lookahead_Discrete_3D(self.path, self.position, self.speed*self.Tstar, 0, 0)
		# print("L2: [%f, %f, %f]" % (p_L2[0], p_L2[1], p_L2[2]))
		# print("robot: [%f, %f, %f]" % (self.position[0], self.position[1], self.position[2]))
		# L2_vec = p_L2 - self.position
		# print("L2-robot: [%f, %f, %f]" % (L2_vec[0], L2_vec[1], L2_vec[2]))
		# print("velocity: [%f, %f, %f]" % (velocity[0,0], velocity[1,0], velocity[2,0]))

		# Generate a lateral acceleration command from the lookahead point
		if self.controller_type == 'trajectory_shaping':
			a_cmd = guidance.trajectory_Shaping_Guidance(np.array([p_L2[0], p_L2[1]]), np.array([self.position[0], self.position[1]]), \
													 np.array([velocity[0,0], velocity[1,0]]), np.array([v_L2[0], v_L2[1]]))
			chi_dot = -a_cmd/self.speed
		else:
			if (self.vehicle_type == 'ground'):
				a_cmd = guidance.L2_Plus_Guidance_2D(np.array([p_L2[0], p_L2[1]]), np.array([self.position[0], self.position[1]]), \
													 np.array([velocity[0,0], velocity[1,0]]), self.Tstar, 0)
				chi_dot = -a_cmd/self.speed
			else:
				a_cmd = guidance.L2_Plus_Guidance_3D(p_L2, self.position, velocity, self.Tstar, 0)
				# Convert lateral acceleration to angular acceleration about the z axis
				chi_dot = -a_cmd[1]/self.speed

		

		# Update class members
		self.L2 = p_L2
		self.command.linear.x = 1.0
		self.command.angular.z = chi_dot
		return

	def __init__(self, name='X1', vehicle_type='ground', controller_type='L2', speed=1.0):
		# Set controller specific parameters
		self.name = name; # robot name
		self.vehicle_type = vehicle_type; # vehicle type (ground vs air)
		self.controller_type = controller_type; # Type of guidance controller from guidance
		self.speed = 1.0 # m/s
		self.Tstar = 1.0 # seconds

		# Booleans for first subscription receive
		self.positionUpdated = 0
		self.pathUpdated = 0

		# Initialize ROS node and Subscribers
		node_name = self.name + '_guidance_controller'
		rospy.init_node(node_name)
		rospy.Subscriber('/gazebo/link_states', LinkStates, self.getPosition)
		self.path = np.empty((3,0))
		rospy.Subscriber('/' + name + '/planned_path', Path, self.getPath)

		# Initialize Publisher topic
		self.pubTopic = '/' + name + '/cmd_vel';
		self.pub = rospy.Publisher(self.pubTopic, Twist, queue_size=10)
		# self.pub = rospy.Publisher('/X1/cmd_vel', Twist, queue_size=10)

		# Initialize twist object for publishing
		self.command = Twist()
		self.command.linear.x = 0.0
		self.command.linear.y = 0.0
		self.command.linear.z = 0.0
		self.command.angular.x = 0.0
		self.command.angular.y = 0.0
		self.command.angular.z = 0.0

	def start(self):
		rate = rospy.Rate(10.0) # 10Hz
		while not rospy.is_shutdown():
			rate.sleep()
			self.updateCommand()
			self.pub.publish(self.command)
		return


if __name__ == '__main__':
	num_args = len(sys.argv)
	# if (num_args != 4):
		# print("Node requires 4 inputs arguments")

	controller = guidance_controller(name=sys.argv[1], vehicle_type=sys.argv[2], \
									 controller_type=sys.argv[3], speed=sys.argv[4])

	try:
		controller.start()
	except rospy.ROSInterruptException:
		pass
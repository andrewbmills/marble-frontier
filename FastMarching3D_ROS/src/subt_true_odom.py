#!/usr/bin/env python
import sys
import numpy as np
import guidance
import rospy
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry

class LinkStateToOdometry:
	def getLinkState(self, data): # Position subscriber callback function
		# Find the index of the link_state
		if (self.link_id == -1):
			i = 0
			for name in data.name[:]:
				if name == self.link_name:
					self.link_id = i
					print("link_id = %d" % self.link_id)
				i = i+1
		# Get the link state data
		if (self.link_id == -1):
			print('Could not find robot state information in /gazebo/link_states/')
		else:
			self.Odometry.pose.pose = data.pose[self.link_id]
			# self.Odometry.pose.pose.position.z = self.Odometry.pose.pose.position.z + 0.3
			self.Odometry.pose.pose.position.z = self.Odometry.pose.pose.position.z
			self.Odometry.twist.twist = data.twist[self.link_id]
		
		# Add time stamp
		self.Odometry.header.stamp = rospy.Time.now()

		return

	def start(self):
		rate = rospy.Rate(25.0) # 50Hz
		while not rospy.is_shutdown():
			rate.sleep()
			self.pub1.publish(self.Odometry)
		return

	def __init__(self, link_name="base_link", topic_name="odometry", robot_name="X1", frame="uav", child_frame="uav"):
		
		node_name = topic_name+ "_" + robot_name
		rospy.init_node(node_name)

		self.link_name = robot_name + "::" + robot_name + "/" + link_name
		self.pubTopic1 = "/" + robot_name + "/" + topic_name
		self.pub1 = rospy.Publisher(self.pubTopic1, Odometry, queue_size=10)

		# Initialize Odometry message object
		self.Odometry = Odometry()
		self.Odometry.header.seq = 1
		self.Odometry.header.frame_id = frame
		self.Odometry.child_frame_id = child_frame

		# Initialize Gazebo LinkState message subscriber
		self.link_id = -1
		rospy.Subscriber('/gazebo/link_states', LinkStates, self.getLinkState)

if __name__ == '__main__':
	publish_tool = LinkStateToOdometry()

	try:
		publish_tool.start()
	except rospy.ROSInterruptException:
		pass
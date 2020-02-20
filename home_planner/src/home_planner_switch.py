#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path

class Switch:
	def getPath(self, msg, num): # Path callback
		if (num == 1):
			self.path1 = msg
		if (num == 2):
			self.path2 = msg
		self.first_path = true
		return

	def getPoint(self, msg, num): # Point callback
		if (num == 1):
			self.point1 = msg
		if (num == 2):
			self.point2 = msg
		self.first_point = true
		return

	def getTask(self, msg)
		if ((msg.data == "Explore") or (msg.data == "Able to plan")):
			self.switch = 1
		if ((msg.data == "Unable to plan"))
			self.switch = 2
		return

	def __init__(self):
		# Initialize ROS node and Subscribers
		node_name = 'home_planner'
		rospy.init_node(node_name)

		self.switch = 1
		self.first_point = False
		self.first_path = False

		# Subscribers
		rospy.Subscriber('path1', Path, self.getPath, 1)
		rospy.Subscriber('path2', Path, self.getPath, 2)
		rospy.Subscriber('point1', Point, self.getPoint, 1)
		rospy.Subscriber('point2', Point, self.getPoint, 2)
		rospy.Subscriber('task', String, self.getTask)

		# Publishers
		self.pub1 = rospy.Publisher("planned_path", Path, queue_size=1)
		self.pub2 = rospy.Publisher("lookahead_point", PointStamped, queue_size=1)

	def start(self):
		rate = rospy.Rate(5.0) # 10Hz
		while not rospy.is_shutdown():
			rate.sleep()
			if (self.first_point):
				if (self.switch == 1): self.pub1.publish(self.point1)
				if (self.switch == 2): self.pub1.publish(self.point2)
			if (self.first_path):
				if (self.switch == 1): self.pub2.publish(self.path1)
				if (self.switch == 2): self.pub2.publish(self.path2)
		return

if __name__ == '__main__':
	home_planner = HomePlanner()
	try:
		home_planner.start()
	except rospy.ROSInterruptException:
		pass

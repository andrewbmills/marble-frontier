#!/usr/bin/env python
import sys
import rospy
from nav_msgs.msg import *
from geometry_msgs.msg import *

class OdomPublisher:
	def getOdomPosition(self, msg):
		self.Odometry.pose.pose.position.x = msg.point.x
		self.Odometry.pose.pose.position.y = msg.point.y
		# self.Odometry.pose.pose.position.z = msg.point.z
		self.Odometry.pose.pose.position.z = -0.2
		self.Odometry.header.stamp = msg.header.stamp
		self.pubPoint = True
		return

	def start(self):
		rate = rospy.Rate(10.0) # 50Hz
		while not rospy.is_shutdown():
			rate.sleep()
			if self.pubPoint:
				self.pub1.publish(self.Odometry)
		return

	def __init__(self):
		node_name = "odom_publisher"
		rospy.init_node(node_name)
		self.pubTopic1 = "/Base/odometry"
		self.pub1 = rospy.Publisher(self.pubTopic1, Odometry, queue_size=10)

		rospy.Subscriber("/clicked_point", PointStamped, self.getOdomPosition)
		# Initialize Odometry message object
		self.Odometry = Odometry()
		self.Odometry.header.seq = 1
		self.Odometry.header.frame_id = "world"
		self.Odometry.child_frame_id = "base_link"
		self.pubPoint = False

if __name__ == '__main__':
	publish_tool = OdomPublisher()

	try:
		publish_tool.start()
	except rospy.ROSInterruptException:
		pass
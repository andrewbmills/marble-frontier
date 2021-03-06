#!/usr/bin/env python
import sys
import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
# from home_planner.msg import PoseGraph
from lcd_pkg.msg import PoseGraph

class HomePlanner:
	def getTransform(self, frame1, frame2, stamp): # Position subscriber callback function
		try:
			(trans,rot) = self.tf_listener.lookupTransform(frame1, frame2, stamp)
			return trans
		except:
			return

	def getPoseList(self, msg): # Pose List subscriber call function
		if (len(msg.poseArray) == 0): return
		self.first_node_list = True
		self.node_list_msg = msg.poseArray
		return

	# def getPoseList(self): # Pose List service call function
	# 	rospy.wait_for_service('trajectory_query')
	# 	try:
	# 		self.first_node_list = True
	# 		trajectory_query = rospy.ServiceProxy('trajectory_query', TrajectoryQuery)
	# 		msg = trajectory_query()
	# 		self.node_list_msg = msg.trajectory
	# 		print(msg.trajectory)
	# 	except rospy.ServiceException, e:
	# 		print("Service call failed: %s" % e)
	# 	return

	def updatePath(self):
		if ((not self.first_node_list) or len(self.node_list_msg) == 0):
			return
		self.path_msg = Path()
		self.path_msg.header.stamp = self.node_list_msg[-1].header.stamp
		self.path_msg.header.frame_id = self.frame_path
		for i in np.arange(0, len(self.node_list_msg), 10):
			new_pose = PoseStamped()
			new_pose.header.frame_id = self.frame_path
			delta = self.getTransform(self.frame_node_list, self.frame_path, self.node_list_msg[i].header.stamp)
			new_pose.pose.position.x = self.node_list_msg[i].pose.position.x + delta[0]
			new_pose.pose.position.y = self.node_list_msg[i].pose.position.y + delta[1]
			new_pose.pose.position.z = self.node_list_msg[i].pose.position.z + delta[2]
			# self.path_msg.poses.insert(0, new_pose)
			self.path_msg.poses.append(new_pose)
		return

	def __init__(self):
		self.first_node_list = False

		# Initialize ROS node and Subscribers
		node_name = 'home_planner'
		rospy.init_node(node_name)

		# Subscribers
		rospy.Subscriber('pose_graph', PoseGraph, self.getPoseList)

		# Params
		self.frame_path = rospy.get_param('home_planner/frame_path', 'world')
		self.frame_node_list = rospy.get_param('home_planner/frame_node_list', 'map')
		self.point_separation_distance = rospy.get_param('home_planner/point_separation_distance', 0.2)

		# Publishers
		self.pub1 = rospy.Publisher("path", Path, queue_size=1)
		self.path_msg = Path()
		self.path_msg.header.frame_id = self.frame_path

		# Initialize tf listener
		self.tf_listener = tf.TransformListener()

	def start(self):
		rate = rospy.Rate(1.0) # 10Hz
		while not rospy.is_shutdown():
			rate.sleep()
			self.updatePath()
			if (self.first_node_list): self.pub1.publish(self.path_msg)
		return

if __name__ == '__main__':
	home_planner = HomePlanner()
	try:
		home_planner.start()
	except rospy.ROSInterruptException:
		pass

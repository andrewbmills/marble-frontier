#!/usr/bin/env python
import sys
import rospy
from topic_tools.srv import MuxSelect
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path

class Switch:
	def getTask(self, msg):
		if ((msg.data == "Explore") or (msg.data == "Able to plan home") or (msg.data == "guiCommand")) and (self.switch == 2):
			self.switch = 1
			rospy.wait_for_service('mux_home_path/select', 5)
			try:
				select_srv = rospy.ServiceProxy('mux_home_path/select', MuxSelect)
				select_srv("planned_path_original")
			except rospy.ServiceException:
				pass
			rospy.wait_for_service('mux_lookahead1/select', 5)
			try:
				select_srv = rospy.ServiceProxy('mux_lookahead1/select', MuxSelect)
				select_srv("lookahead_point_original")
			except rospy.ServiceException:
				pass
			rospy.wait_for_service('mux_lookahead2/select', 5)
			try:
				select_srv = rospy.ServiceProxy('mux_lookahead2/select', MuxSelect)
				select_srv("lookahead_vec_original")
			except rospy.ServiceException:
				pass

		if ((msg.data == "Unable to plan home")) and (self.switch == 1):
			self.switch = 2
			rospy.wait_for_service('mux_home_path/select', 5)
			try:
				select_srv = rospy.ServiceProxy('mux_home_path/select', MuxSelect)
				select_srv("home_path")
			except rospy.ServiceException:
				pass
			rospy.wait_for_service('mux_lookahead1/select', 5)
			try:
				select_srv = rospy.ServiceProxy('mux_lookahead1/select', MuxSelect)
				select_srv("home_planner/lookahead_point")
			except rospy.ServiceException:
				pass
			rospy.wait_for_service('mux_lookahead2/select', 5)
			try:
				select_srv = rospy.ServiceProxy('mux_lookahead2/select', MuxSelect)
				select_srv("home_planner/lookahead_vec")
			except rospy.ServiceException:
				pass

		return

	def __init__(self):
		# Initialize ROS node and Subscribers
		node_name = 'home_planner_switch'
		rospy.init_node(node_name)

		self.switch = 1
		rospy.Subscriber('task', String, self.getTask)

	def start(self):
		rate = rospy.Rate(5.0) # 10Hz
		while not rospy.is_shutdown():
			rate.sleep()
		return

if __name__ == '__main__':
	home_planner_switch = Switch()
	try:
		home_planner_switch.start()
	except rospy.ROSInterruptException:
		pass

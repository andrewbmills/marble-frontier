#!/usr/bin/env python
import sys
import numpy as np
import rospy
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField

class markerToPcl2:
	def getMarker(self, data):
		self.stamp = data.markers[0].header.stamp
		self.frame_id = data.markers[0].header.frame_id
		points = np.empty([0])
		for point in data.markers[0].points:
			points = np.append(points, point.x)
			points = np.append(points, point.y)
			points = np.append(points, point.z)

		self.points = points	
		self.newMarker = True
		print(np.size(self.points))
		return

	def updatePcl2(self):
		points = self.points
		if self.newMarker:
			if self.stamp:
				self.pcl2.header.stamp = self.stamp
			if self.frame_id:
				self.pcl2.header.frame_id = self.frame_id

			self.pcl2.row_step = self.pcl2.point_step*len(points)
			self.pcl2.data = np.asarray(points, np.float32).tostring()
			self.pcl2.width = len(points)/3.0
		return

	def __init__(self, markerTopic, pclTopic="/X1/occGrid"):
		# Get input arguments
		nodeName = "markerToPcl2"

		# Start Node with node_name as its name
		rospy.init_node(nodeName)

		# Start Subscriber to topic name markerTopic of type MarkerArray with callback function self.getMarker()
		rospy.Subscriber(markerTopic, MarkerArray, self.getMarker)
		self.points = np.empty([0])

		# Start Publisher to topic name pclTopic of type PointCloud2
		self.pub = rospy.Publisher(pclTopic, PointCloud2, queue_size=10)

		# Initialize PointCloud2 object for publishing
		self.pcl2 = PointCloud2()
		self.stamp = None
		self.frame_id = None
		self.seq = None
		self.pcl2.fields = [
			PointField('x', 0, PointField.FLOAT32, 1),
			PointField('y', 4, PointField.FLOAT32, 1),
			PointField('z', 8, PointField.FLOAT32, 1)
		]
		self.pcl2.is_bigendian = False
		self.pcl2.point_step = 12
		self.pcl2.is_dense = True
		self.pcl2.height = 1

		# Track whether or not the marker array has been updated
		self.newMarker = False

		return

	def start(self):
		rate = rospy.Rate(2.0) # 2Hz
		while not rospy.is_shutdown():
			rate.sleep()
			self.updatePcl2()
			self.pub.publish(self.pcl2)
		return

if __name__ == '__main__':
	num_args = len(sys.argv)
	if (num_args == 2):
		print("Publishing pointCloud2 topic with name /X1/occGrid.")
		converter = markerToPcl2(markerTopic=sys.argv[1])
	elif (num_args <= 1):
		print("Node requires an input topic name.")
		pass
	else:
		converter = markerToPcl2(markerTopic=sys.argv[1], pclTopic=sys.argv[2])

	try:
		converter.start()
	except rospy.ROSInterruptException:
		pass
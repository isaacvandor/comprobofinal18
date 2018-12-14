#!/usr/bin/env python

from __future__ import print_function
from std_msgs.msg import Header
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
import random
import ast

class marker_viz(object):
	def __init__(self):
		"initializes marker node"
		rospy.init_node('marker_viz')
		self.rate = rospy.Rate(10)
		self.num_particles = 10
		self.particles = None
		rospy.Subscriber('cluster_string', String, self.process_clusters)
		rospy.Subscriber('/circle_string', String, self.process_circles)
		self.pub = rospy.Publisher('visualization_marker', MarkerArray, queue_size = 10)
		self.cluster_array = []
		self.max_markers = 0
		self.circle_array = []

	def process_circles(self, message):
		"process detected circles"
		self.circle_array = ast.literal_eval(message.data)
		if(self.circle_array != 0):
			self.circle_radius = self.circle_array[0]
			self.circle_array = self.circle_array[1]
		print(self.circle_array)

	def process_clusters(self, message):
		"processes cluster string"
		self.cluster_array = ast.literal_eval(message.data)

	def create_circle_markers(self):
		"creates circle markers from circle array"
		#check if we have points in circle array
		if(self.circle_array != 0):
			for i, point in enumerate(self.circle_array):
				if(i == 0):
					color_array = [1,0,0] #red for first bucket
				else:
					color_array = [0,0,1] #blue for second bucket
				#add marker with circle
				self.create_marker(point[0], point[1], color_array, 1, 1)
				self.marker.id = self.id_number
				self.markerArray.markers.append(self.marker)
				self.id_number += 1

	def create_cluster_markers(self):
		"creates cluster markers"
		self.markerArray = MarkerArray()
		self.markerArray.markers = []
		self.id_number = 0
		#go through each cluster in cluster array to add marker point
		for i, cluster in enumerate(self.cluster_array):
			#run through cluster arrays
			for point in cluster:
				if(i == 0): #hard coded colors for markers up to 8.
					color_array = [1, 0, 0]
				elif(i == 1):
					color_array = [0, 0, 1]
				elif(i == 2):
					color_array = [0, 1, 0]
				elif(i == 3):
					color_array = [0, 1, 1]
				elif(i == 4):
					color_array = [1, 1, 0]
				elif(i == 5):
					color_array = [1, 0, 1]
				elif(i == 6):
					color_array = [5, .5, .3]
				elif(i == 7):
					color_array = [.5, .3, .5]
				elif(i == 8):
					color_array = [.3, .5 ,.5]
				else:
					color_array = [1, 1, 1]
				self.create_marker(point[0], point[1], color_array, 1, 0)
				self.marker.id = self.id_number
				self.markerArray.markers.append(self.marker)
				self.id_number += 1
		
	def clear_excess_markers(self):
		"updates markers that should not be there"
		#if we update, but have more markers than id_numbers
		if(self.max_markers > self.id_number):
			#fill with empty marker
			for i in range(id_number, self.max_markers):
				self.create_marker(0,0,[0,0,0], 0, 0)
				self.marker.id = i
				self.markerArray.markers.append(self.marker)
		self.max_markers = self.id_number


	def create_marker(self, x,y, color_array, add, marker_type):
		"creates marker with position x,y, color, and add or subtract"
		self.marker = Marker()
		self.marker.header.frame_id = "map"
		#check if add is true, delete or add coressponding marker
		if(add == 1):
			self.marker.action = self.marker.ADD
		else:
			self.marker.action = self.marker.DELETE
		if(marker_type == 0): #0  = point, 1 = circle
			self.marker.type = self.marker.SPHERE
			scale = .1
			self.marker.scale.x = scale
			self.marker.scale.y = scale
			self.marker.scale.z = scale
		elif(marker_type == 1):
			self.marker.type = self.marker.CYLINDER
			scale = .3
			self.marker.scale.x = scale
			self.marker.scale.y = scale
			self.marker.scale.z = .1
		self.marker.pose.position.x = x
		self.marker.pose.position.y = y
		self.marker.pose.position.z = 0
		self.marker.color.a = 1
		self.marker.color.r = color_array[0]
		self.marker.color.g = color_array[1]
		self.marker.color.b = color_array[2]

	def run(self):
		"runs marker viz"
		while not rospy.is_shutdown():
			self.create_cluster_markers()
			self.create_circle_markers()
			self.clear_excess_markers()
			self.pub.publish(self.markerArray)
			self.rate.sleep()

if __name__ == '__main__':
	node = marker_viz()
	node.run()
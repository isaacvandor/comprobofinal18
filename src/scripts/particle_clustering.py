#!/usr/bin/env python

from __future__ import print_function
from std_msgs.msg import Header
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from sklearn.cluster import DBSCAN
import ast
import rospy
import math
import random


class particle_clustering(object):
	def __init__(self):
		"initialize particle clustering"
		rospy.init_node('particle_clustering')
		rospy.Subscriber('/scan', LaserScan, self.process_scan)
		rospy.Subscriber('/map_string', String, self.process_map)
		self.pub = rospy.Publisher('cluster_string', String, queue_size = 10)
		self.rate = rospy.Rate(10)
		self.clusters = [] #array of array of tuples
		self.particle_array = []
		self.cluster_array = []
		self.map_array = []

	def process_map(self, message):
		map_array = ast.literal_eval(message.data)
		self.map_array = map_array[1:]

	def process_scan(self, message):
		"""take in scan data, returns x and y values in baselink ref frameself.
		   omits points with 0 range value."""
		   #Note: currently unused
		ranges = message.ranges
		xy_array = []
		for i, dist in enumerate(ranges):
			if dist != 0:
				theta = math.radians(i+90)
				x = math.cos(theta)*dist
				y = math.sin(theta)*dist
				xy_array.append((x,y))
		self.particle_array = xy_array

	def create_clusters(self, point_array):
		"creates density based clusters based on xy points"
		if(len(point_array) > 0): #if point_array is not empty
			X = np.array(point_array) #numpyize it
			clustering = DBSCAN(eps = 0.1, min_samples = 2).fit(X)
			labels = clustering.labels_ #array of number that represents point cluster
			n_clusters = len(set(labels)) - (1 if -1 in labels else 0) #number of clusters minus noise
			cluster_array = [[] for c in range(n_clusters + 1)] #create empty lists
			if(len(labels) == len(point_array)): #counters some weird end case where lists arnt same length
				for i, particle in enumerate(point_array):
					#add particles to cluster_array depending on cluster label
					current_label = labels[i]+1
					cluster_array[current_label].append(particle)
				self.cluster_array = cluster_array

	def run(self):
		"Run clustering"
		while not rospy.is_shutdown():
			self.create_clusters(self.map_array)
			cluster_string = str(self.cluster_array)
			#publishes cluster_array as a string
			self.pub.publish(cluster_string)
			self.rate.sleep()

if __name__ == '__main__':
	node = particle_clustering()
	node.run()

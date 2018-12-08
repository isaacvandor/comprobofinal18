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
		self.map_array = ast.literal_eval(message.data)

	def process_scan(self, message):
		"""take in scan data, returns x and y values in baselink ref frameself.
		   omits points with 0 range value."""
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
		if(len(point_array) > 0):
			X = np.array(point_array)
			#print(X)
			clustering = DBSCAN(eps = 0.1, min_samples = 2).fit(X)
			labels = clustering.labels_ #array of number that represents point cluster
			print(labels)
			n_clusters = len(set(labels)) - (1 if -1 in labels else 0) #number of clusters minus noise
			cluster_array = [[] for c in range(n_clusters + 1)] #create empty lists
			#print(labels)
			print(point_array)
			if(len(labels) == len(point_array)):
				for i, particle in enumerate(point_array):
					#add particles to cluster_array depending on cluster label
					current_label = labels[i]+1
					#print(n_clusters, current_label)
					cluster_array[current_label].append(particle)
				self.cluster_array = cluster_array
				#for i in range(n_clusters):
					#print(self.cluster_array[i])
				
	def run(self):
		"Run clustering"
		while not rospy.is_shutdown():
			self.create_clusters(self.map_array)
			#test = str([[(1, 3), (5, 3)], [(2, 4), (7, 4)]])
			cluster_string = str(self.cluster_array)
			self.pub.publish(cluster_string)
			#print(cluster_string)
			self.rate.sleep()

if __name__ == '__main__':
	node = particle_clustering()
	node.run()


# particle clustering functions
# take all x-y coordinates, grab particles
# returns object? cluster 1 - array of particles?
# particle.clusters(1) = tuple of x,y
# send to marker script
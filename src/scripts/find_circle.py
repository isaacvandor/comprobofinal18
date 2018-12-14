#!/usr/bin/env python

from __future__ import print_function
from geometry_msgs.msg import PointStamped, PointStamped, Twist
from std_msgs.msg import Header
from std_msgs.msg import String
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import time, math, rospy
import numpy as np
import ast

class FindCircle(object):
    """ROS node that receives a map and identifies the center coordinates of
    circles of a known radius. Publishes the spatial coordinates and pixel
    coordinates of the circles (buckets). Can be implemented live on a neato
    as well."""

    def __init__(self):
        # initialize ROS things - subs/pubs/etc.
        rospy.init_node("FindCircle")
        self.rate = rospy.Rate(10)  # set publish rate

        # subscribers and publishers
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        rospy.Subscriber('cluster_string', String, self.process_cluster)
        rospy.Subscriber('map_string', String, self.process_map)
        self.pub = rospy.Publisher('/circle_string', String, queue_size=10)
        self.pub1 = rospy.Publisher('/pixels_string', String, queue_size=10)

        # initialize a bunch of useful variables
        self.go = True # used to watch the bump sensor
        self.point_cloud = None # map point cloud
        self.live_point_cloud = None # live scan point cloud
        self.coms = None # cluster centers of mass
        self.max_dim = None # maximum dimension of laser scan
        self.map_res = None # resolution of the map, meters/pixel
        self.bucket_pix = None # pixel coordinates of the buckets
        self.vis_msg = [] # message storage for visualization

    def process_scan(self, m):
        """callback function triggered on the laser scan subscriber for live
        neato testing. cleans out all 0 values. stores points as (x,y) tuples"""
        max_r = 1.0
        ranges = m.ranges
        point_cloud = []

        # clean and convert to cartesian
        for i in range(len(ranges)):
            if ranges[i] != 0 and ranges[i]<max_r:
                theta = math.radians(i+90)
                r = ranges[i]
                x = math.cos(theta)*r
                y = math.sin(theta)*r
                point_cloud.append((x,y)) # append cartesian tuple

        self.max_dim = abs(max(max(point_cloud))) # largest x or y value in scan
        self.live_point_cloud = point_cloud

    def process_map(self,message):
        """callback function for processing the map data from other node. The
        first value in the list is the resolution, and all further values are
        (x,y) tuples of map points."""
        map_array = ast.literal_eval(message.data)
        self.map_res = map_array[0] # meters/pixel
        self.point_cloud = map_array[1:]

    def process_cluster(self,message):
        """receives an list of lists from the clustering node and processes
        to determine where in the map to look for the target. The first list is
        always a noise element and is not a clustered set of points."""
        COMs = []
        cluster_array = ast.literal_eval(message.data)  # convert string

        # sort through clusters, find CoM, ignore first list
        for cluster in cluster_array[1:]:
            xs = [c[0] for c in cluster]
            ys = [c[1] for c in cluster]
            com_x, com_y = sum(xs)/len(xs), sum(ys)/len(ys) # compute CoM
            COMs.append((com_x, com_y))

        self.coms = COMs    # store for later

    def make_template(self, r, resolution, center=(0,0)):
        """returns a list of points around a circle of given radius and
        angular resolution between points. Default center (0,0)."""
        thetas = np.linspace(0,2*math.pi, 360/resolution)
        circle_points = [(r*math.cos(theta)+center[0],r*math.sin(theta)+center[1]) for theta in thetas]
        x_val = [x[0] for x in circle_points]
        y_val = [x[1] for x in circle_points]

        return(circle_points)

    def scan_cluster_locations(self, centers, dim, grid_size):
        """takes in a list of cluster centers and populates a grid of points
        centered at each cluster to account for noise and error in both
        scanning and clustering."""
        all_points = [] # all points in all grids

        # create grids centered at cluster center
        for center in centers:
            c_x = center[0]
            c_y = center[1]
            mini_grid = np.linspace(-dim, dim, grid_size)

            # create final list, offset by cluster center coordinates
            scan_points = [(x+c_x, y+c_y) for x in mini_grid for y in mini_grid]

            for point in scan_points:
                all_points.append(point)

        return all_points

    def min_dist(self, test, points, template):
        """calculates the weight value where the location "test" (x,y) is the
        center of the circle defined by "template" given point cloud "points".
        Template argument is centered at (0,0)."""

        weight = 0  # cumulative weight of test point

        # translate template to testing points
        new_template = [(temp[0]+test[0], temp[1]+test[1]) for temp in template]

        # loop through point cloud to find min dist
        for point in points:
            dists = []
            px = point[0]
            py = point[1]

            for temp in new_template:
                tx = temp[0]
                ty = temp[1]
                dist = math.sqrt((px-tx)**2+(py-ty)**2) # calculate distance

                # handle identical point case to avoid divide by 0 error
                if dist == 0:
                    dists.append(0.01)
                else:
                    dists.append(dist)

            weight += 1/min(dists)  # sum inverse of distance to weight
        return weight

    def run_points(self, scan_set, points, template):
        """loop through and weight all of the points in our scan set. returns
        a list of all the weights."""
        all_weights = []

        for scan_point in scan_set:
            scan_weight = self.min_dist(scan_point, points, template)
            all_weights.append((scan_point, scan_weight))
        return(all_weights)

    def run(self):
        """main run loop, calculates the highest weighted points and stores
        important information for publishing"""
        while not rospy.is_shutdown():
            if self.coms != None and self.point_cloud != None:
                print('messages received! started processing')
                bucket_r = 0.055 # meters, bucket radius
                bucket_d = bucket_r*2 # meters, bucket diameter

                map_points = self.point_cloud # most recent map message
                centers = self.coms # most recent cluster message

                template = self.make_template(bucket_d,10) # generate template
                scan_grid = centers # scan only cluster centers

                # run all points
                all_weights = self.run_points(scan_grid, map_points, template)

                # post process, sort by weight
                new_weights = sorted(all_weights, key=lambda x: x[1], reverse=True)
                best_centers = [x[0] for x in new_weights[0:2]]

                # some matplotlib visualization
                map_x = [x[0] for x in map_points] # map
                map_y = [x[1] for x in map_points]
                c_x = [x[0] for x in centers] # cluster centers
                c_y = [x[1] for x in centers]
                plt.plot(map_x, map_y, 'r.', markersize=5)

                # plot best weights and translated centers
                for weight in best_centers:
                    c_temp_x = weight[0]
                    c_temp_y = weight[1]
                    template_x = [x[0]+c_temp_x for x in template]
                    template_y = [x[1]+c_temp_y for x in template]
                    plt.plot(c_temp_x, c_temp_y, 'b.', markersize=10)
                    plt.plot(template_x, template_y, 'g.')

                # publishing information for A*
                self.bucket_pix = [(x[0]/self.map_res, x[1]/self.map_res) for x in best_centers]
                self.vis_msg.append(bucket_d/2)
                self.vis_msg.append(best_centers)
                plt.show()

                break

    def send_info(self):
        """send pixel values to a rostopic"""
        while not rospy.is_shutdown():
            self.pub1.publish(str(self.bucket_pix)) # pixels for A*
            self.pub.publish(str(self.vis_msg))     # for Rviz
            self.rate.sleep()

if __name__ == '__main__':
    node = FindCircle()
    node.run()
    node.send_info()

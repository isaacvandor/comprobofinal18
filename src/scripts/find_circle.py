#!/usr/bin/env python

from __future__ import print_function
from geometry_msgs.msg import PointStamped, PointStamped, Twist
from std_msgs.msg import Header
from std_msgs.msg import String
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import statistics
import time, math, rospy
import numpy as np
import ast

class FindCircle(object):
    """given a known radius and a neato's stationary point cloud, find
    the highest probability coordinates for the circle (base link)."""

    def __init__(self):
        # initialize ROS things - subs/pubs/etc.
        rospy.init_node("FindCircle")

        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        rospy.Subscriber('/bump', Bump, self.process_bump)
        rospy.Subscriber('cluster_string', String, self.process_cluster)
        self.pub = rospy.Publisher('/circle_string', String, queue_size=10)
        # self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.go = True # used to watch the bump sensor
        self.curr_point_cloud = None # live updating point cloud array
        self.coms = None # live updating cluster coms
        self.max_dim = None

    def process_bump(self, m):
        """callback function triggered on the bump subscriber. Used as an easy
        e-stop mechanism."""
        lf = m.leftFront
        rf = m.rightSide
        # if either bunp sensor is triggered
        if lf != 0 or rf != 0:
            self.pub.publish(self.stop)
            self.go = False

    def process_scan(self, m):
        """callback function triggered on the laser scan subscriber. cleans out
        all 0 values."""
        max_r = 1.0
        ranges = m.ranges
        point_cloud = []

        for i in range(len(ranges)):
            if ranges[i] != 0 and ranges[i]<max_r:
                theta = math.radians(i+90)
                r = ranges[i]
                x = math.cos(theta)*r
                y = math.sin(theta)*r
                point_cloud.append((x,y)) # append cartesian tuple

        self.max_dim = abs(max(max(point_cloud))) # largest x or y value in scan
        self.curr_point_cloud = point_cloud

    def process_cluster(self,message):
        """receives an array of arrays from the clustering node and processes
        to determine where in the map to look for the target. The first list is
        always a noise element and is not a clustered set of points."""
        COMs = []
        cluster_array = ast.literal_eval(mesage.data)

        # sort through clusters, find COM, ignore first array for now
        for cluster in cluster_arrays[1:]:
            xs = [c[0] for c in cluster]
            ys = [c[1] for c in cluster]
            com_x, com_y = sum(xs)/len(xs), sum(ys)/len(ys);
            COMs.append((com_x, com_y))

        self.coms = COMs

    def make_template(self, r, resolution, center=(0,0)):
        """creates a point cloud representing a circle of given radius and
        angular resolution between points. Default center (0,0).

        returns list of points"""
        thetas = np.linspace(0,2*math.pi, 360/resolution)
        circle_points = [(r*math.cos(theta)+center[0],r*math.sin(theta)+center[1]) for theta in thetas]
        x_val = [x[0] for x in circle_points]
        y_val = [x[1] for x in circle_points]

        # plt.plot(x_val,y_val, 'r.')
        # plt.axis('equal')
        # plt.show()

        return(circle_points)

    def scan_these_locations(self, dim, resolution):
        """creates list of points to check based on max dimension in the laser
        scan and a given point to point resolution."""
        list = np.linspace(-dim, dim, 20)
        scan_points = [(x,y) for x in list for y in list]
        return scan_points

    def scan_cluster_locations(self, centers, radius, resolution, error=0.125):
        """creates a list of points to check based on mini-grids at cluster
        centers. error is a parameter that expands the grid dimensions."""
        all_points = [] # all points in all grids

        # create grids centered at cluster center
        for center in center:
            c_x = center[0]
            c_y = center[1]
            dim = radius+error
            mini_grid = np.linspace(-dim,dim)

            # create final list, offset by cluster center coordinates
            scan_points = [(x+c_x, y+c_y) for x in list for y in list]

            for point in scan_points:
                all_points.append(point)

        return all_points

    def min_dist(self, test, points, template):
        """calculates the weight value for the location "test" (x,y) is the
        center of the circle defined by "template" given point cloud "points".
        Template argument is centered at (0,0)."""

        weight = 0  # cumulative weight of test point

        # translate template to testing points
        new_template = [(temp[0]+test[0], temp[1]+test[1]) for temp in template]

        # x_val = [x[0] for x in new_template]
        # y_val = [x[1] for x in new_template]
        # plt.plot(x_val,y_val, 'r.')
        # plt.axis('equal')
        # plt.show()

        # loop through points to find min dist
        for point in points:
            dists = []
            px = point[0]
            py = point[1]

            for temp in new_template:
                tx = temp[0]
                ty = temp[1]
                dist = math.sqrt((px-tx)**2+(py-ty)**2)

                if dist == 0:
                    dists.append(0.01)
                else:
                    dists.append(dist)

            weight += 1/min(dists)
        return weight

    def run_points(self, scan_grid, points, template):
        """loop through all of the points in our map array."""
        all_weights = []
        for scan_point in scan_grid:
            scan_weight = self.min_dist(scan_point, points, template)
            all_weights.append((scan_point, scan_weight))
        return(all_weights)

    def run(self):
        """main run loop"""
        while self.go:
            # checks if list is populated by first scan
            bucket_d = 0.25 # meters, bucket diameter
            c_now = self.coms # most recent cluster message centers
            template = self.make_template(bucket_d,10) # generate template
            scan_grid = self.scan_cluster_locations(centers, bucket_d/2, 10)
            all_weights = self.run_points(scan_grid, laser_scan, template)

            # post process
            new_weights = sorted(all_weights, key=lambda x: x[1], reverse=True)

            for weight in new_weights[0:10]:
                print(weight)
            break

            """PUBLISH STRING OF VISUALIZATION DATA"""

            """
            if isinstance(self.curr_point_cloud, list):
                t_x, t_y = self.points_to_vector(self.xs, self.ys, 1, .2)
                r, theta = self.cart_to_polar(t_x, t_y)
                self.drive_to_target(r, theta)
            """
        # self.pub.publish(self.stop) # if bump sense breaks self.go loop, stop

if __name__ == '__main__':
    node = FindCircle()
    node.run()

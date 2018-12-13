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
    """given a known radius and a neato's stationary point cloud, find
    the highest probability coordinates for the circle (base link)."""

    def __init__(self):
        # initialize ROS things - subs/pubs/etc.
        rospy.init_node("FindCircle")
        self.rate = rospy.Rate(10)

        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        rospy.Subscriber('cluster_string', String, self.process_cluster)
        rospy.Subscriber('map_string', String, self.process_map)
        self.pub = rospy.Publisher('/circle_string', String, queue_size=10)
        self.pub1 = rospy.Publisher('/pixels_string', String, queue_size=10)

        # self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.go = True # used to watch the bump sensor
        self.point_cloud = None # live updating point cloud array
        self.live_point_cloud = None # variable for live neato testing
        self.coms = None # live updating cluster coms
        self.max_dim = None
        self.map_res = None # resolution of the map
        self.bucket_pix = None
        self.vis_msg = []

    def process_scan(self, m):
        """callback function triggered on the laser scan subscriber. cleans out
        all 0 values. for live neato testing"""
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
        self.live_point_cloud = point_cloud

    def process_map(self,message):
        map_array = ast.literal_eval(message.data)
        self.map_res = map_array[0]
        self.point_cloud = map_array[1:]

    def process_cluster(self,message):
        """receives an array of arrays from the clustering node and processes
        to determine where in the map to look for the target. The first list is
        always a noise element and is not a clustered set of points."""
        COMs = []
        cluster_array = ast.literal_eval(message.data)

        # sort through clusters, find COM, ignore first array for now
        for cluster in cluster_array[1:]:
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

    def scan_cluster_locations(self, centers, dim, grid_size):
        """creates a list of points to check based on mini-grids at cluster
        centers. error is a parameter that expands the grid dimensions."""
        all_points = [] # all points in all grids

        # create grids centered at cluster center
        for center in centers:
            c_x = center[0]
            c_y = center[1]
            mini_grid = np.linspace(-dim,dim, grid_size)

            # create final list, offset by cluster center coordinates
            scan_points = [(x+c_x, y+c_y) for x in mini_grid for y in mini_grid]

            for point in scan_points:
                all_points.append(point)

            # scan_x = [x[0] for x in all_points]
            # scan_y = [x[1] for x in all_points]
            # plt.plot(scan_x, scan_y, 'b.', markersize=5)

        return all_points

    def min_dist(self, test, points, template):
        """calculates the weight value for the location "test" (x,y) is the
        center of the circle defined by "template" given point cloud "points".
        Template argument is centered at (0,0)."""

        weight = 0  # cumulative weight of test point

        # translate template to testing points
        new_template = [(temp[0]+test[0], temp[1]+test[1]) for temp in template]

        x_val = [x[0] for x in new_template]
        y_val = [x[1] for x in new_template]
        # plt.plot(x_val,y_val, 'g.')
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
        # main run loop
        while not rospy.is_shutdown():
            if self.coms != None and self.point_cloud != None:
                print('messages received! started processing')
                bucket_r = 0.055 # meters, bucket radius
                bucket_d = bucket_r*2 # meters, bucket diameter

                map_points = self.point_cloud
                centers = self.coms # most recent cluster message centers

                template = self.make_template(bucket_d,10) # generate template
                scan_grid = centers
                # scan_grid = self.scan_cluster_locations(centers, 0.01, 2)
                all_weights = self.run_points(scan_grid, map_points, template)

                # post process, sort by weight
                new_weights = sorted(all_weights, key=lambda x: x[1], reverse=True)

                # refine resolution around top weighted centers
                best_centers = [x[0] for x in new_weights[0:2]]
                # scan_grid = self.scan_cluster_locations(centers, 0.05, 2)

                # some visualization
                map_x = [x[0] for x in map_points]
                map_y = [x[1] for x in map_points]
                c_x = [x[0] for x in centers]
                c_y = [x[1] for x in centers]
                plt.plot(map_x, map_y, 'r.', markersize=5)

                for weight in best_centers:
                    c_temp_x = weight[0]
                    c_temp_y = weight[1]
                    template_x = [x[0]+c_temp_x for x in template]
                    template_y = [x[1]+c_temp_y for x in template]
                    plt.plot(c_temp_x, c_temp_y, 'b.', markersize=10)
                    plt.plot(template_x, template_y, 'g.')
                    print(weight)

                self.bucket_pix = [(x[0]/self.map_res, x[1]/self.map_res) for x in best_centers]
                self.vis_msg.append(bucket_d/2)
                self.vis_msg.append(best_centers)
                print(self.vis_msg)
                # plt.show()

                break

    def send_info(self):
        """send pixel values to a rostopic"""
        while not rospy.is_shutdown():
            self.pub1.publish(str(self.bucket_pix))
            self.pub.publish(str(self.vis_msg))
            self.rate.sleep()
            self.pub.publish


if __name__ == '__main__':
    node = FindCircle()
    node.run()
    node.send_info()

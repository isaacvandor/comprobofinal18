#!/usr/bin/env python

from __future__ import print_function
from scipy.misc import imread, imresize
from nav_msgs.srv import GetMap
from std_msgs.msg import String
import matplotlib.pyplot as plt
import time, math, rospy
import numpy as np


class ImageToPoints(object):
    """ A ros node for converting a YAML map to a point cloud for
    object identification and processing. """

    def __init__(self):
        # grab the map from the map server
        rospy.init_node('image_to_points')
        self.pub = rospy.Publisher('/map_string', String, queue_size=10)
        self.X = None # final data array
        rospy.wait_for_service("static_map")
        static_map = rospy.ServiceProxy("static_map", GetMap)
        self.map = static_map().map
        self.rate = rospy.Rate(10)

        # an array for all x,y coordinates in the map
        X = []
        res = self.map.info.resolution  # meters per pixel
        X.append(res)
        # sort through array
        ind = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                # occupancy grids are stored in row major order
                ind = i + j*self.map.info.width
                if self.map.data[ind] > 0:
                    x = float(i)*res    # multiple pixel
                    y = float(j)*res
                    X.append((x,y))
                    ind += 1

        self.X = X # assign (x, y) array to self
        x_s = [x[0] for x in X[1:]]
        y_s = [x[1] for x in X[1:]]

        # plt.plot(x_s, y_s, 'b.')
        # plt.plot(11.84, 3.57, 'r.', markersize=20)
        # plt.show()

    def send_to_process(self):
        """sends the (x, y) tuple array to the processing script"""
        while not rospy.is_shutdown():
            msg = str(self.X) # convert array to string for easy sending
            # print(self.X)
            self.pub.publish(msg)
            self.rate.sleep()

if __name__ == '__main__':
    node = ImageToPoints()
    node.send_to_process()
    # node.run()

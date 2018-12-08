#!/usr/bin/env python

from __future__ import print_function
from scipy.misc import imread, imresize
from nav_msgs.srv import GetMap
import matplotlib.pyplot as plt
import time, math, rospy
import pandas as pd
import numpy as np


class ImageToPoints(object):
    """ A ros node for converting a YAML map to a point cloud for
    object identification and processing. """

    def __init__(self):
        # grab the map from the map server
        rospy.wait_for_service("static_map")
        static_map = rospy.ServiceProxy("static_map", GetMap)
        self.map = static_map().map

        # an array for all x,y coordinates in the map
        X = np.zeros((self.map.info.width*self.map.info.height, 2))
        res = self.map.info.resolution  # meters per pixel

        # sort through array
        ind = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                # occupancy grids are stored in row major order
                ind = i + j*self.map.info.width
                if self.map.data[ind] > 0:
                    X[ind, 0] = float(i)*res    # multiple pixel
                    X[ind, 1] = float(j)*res
                    ind += 1

        x_s = X[:,0]
        y_s = X[:,1]

        plt.plot(x_s, y_s, 'b.')
        plt.show()

if __name__ == '__main__':
    node = ImageToPoints()
    # node.run()

#!/usr/bin/env python
""" A script for converting the A* coordinate outputs into linear and angular velocities for the Neato"""

from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist, Vector3
from math import *
from numpy import *
from astar import astar


class navstar:
    def __init__(self, pixel, time_step):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.path = pixel
        self.time_step = time_step
        self.converting_factor = 0.05
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.lin_vec = 0.0
        self.ang_vec = 0.0
        self.old_direction = (1,0)
        self.rate = rospy.Rate(5)
        self.commands = []
        self.vel_msg = Twist()
        self.distances = {}
        self.directions = {}
        self.vectors_heading = {}
        self.pixel_step = 10

    def linear_approximation(self):
        """
        Input: pixel_step = Number of pixels to skip for linear approximation of path
        Output: Linear approximation of path using vectors
        """
        for i in range(int(len(self.path) / self.pixel_step) - 1): 
            init_x = self.path[self.pixel_step * i][0]
            init_y = self.path[self.pixel_step * i][1]
            dest_x = self.path[self.pixel_step * (i + 1)][0]
            dest_y = self.path[self.pixel_step * (i + 1)][1]
            heading = (dest_x - init_x, dest_y - init_y)
            self.vectors_heading[i] = heading
        return self.vectors_heading

    def get_vectors(self):
        """
        Input: Linear Approximation
        Output: Vector distances and directions
        """
        headings = self.linear_approximation()
        for i in range(len(headings)):
            x = headings[i][0]
            y = headings[i][1]

            # get distance and direction (s/o to trigonometry)
            distance = math.sqrt(x ** 2 + y ** 2)
            self.distances[i] = distance
            direction = (1 / distance * x, 1 / distance * y)
            self.directions[i] = direction
        #print('dist, dir:', self.distances, self.directions)
        return self.distances, self.directions

    def get_cmd_vel(self):
        """
        Input: Vector distances and directions
        Output: List of linear and angular velocity commands
        """
        self.distances, self.directions = self.get_vectors()
        for i in range(len(self.distances)):
            # find angle to turn
            self.new_direction = self.directions[i]
            angle_old_cos = arccos(self.old_direction[0])
            angle_old_sin = arcsin(self.old_direction[1])
            if angle_old_sin < 0:  # express angle always in 0 to 2pi
                angle_old = 2 * pi - angle_old_cos
            else:
                angle_old = angle_old_cos
            angle_new_cos = arccos(self.new_direction[0])
            angle_new_sin = arcsin(self.new_direction[1])
            if angle_new_sin < 0:
                angle_new = 2 * pi - angle_new_cos
            else:
                angle_new = angle_new_cos
            theta = angle_new - angle_old

            # calculate linear velocity
            self.linear_velocity = self.distances[i] / self.time_step * self.converting_factor

            # calculate angular velocity
            self.angular_velocity = theta / self.time_step

            # Separate into x,y,z components and publish
            self.vel_msg.linear.x = self.linear_velocity
            self.vel_msg.linear.y = 0
            self.vel_msg.linear.z = 0
            self.vel_msg.angular.x = 0
            self.vel_msg.angular.y = 0
            self.vel_msg.angular.z = self.angular_velocity
            self.pub.publish(self.vel_msg)

            #Reset variables to loop through again
            self.old_direction = self.new_direction
            self.rate.sleep()
        else:
            self.lin_vec = Vector3(0,0,0)
            self.ang_vec = Vector3(0,0,0)
            self.pub.publish(Twist(linear=self.lin_vec, angular=self.ang_vec))
            rospy.signal_shutdown("reached target destination...shutting down now") 

if __name__ == '__main__':
    while not rospy.is_shutdown():
        nav = astar()
        coords = nav.run_astar((660,600),(680,495),"../maps/finalnightslam_bestmap.png") #Coords for round trashcan to corner of box
        robot = navstar(coords, 1)
        robot.get_cmd_vel()
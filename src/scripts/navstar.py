#!/usr/bin/env python

from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist, Vector3
from math import *
from numpy import *
from astar import astar


class navstar:
    def __init__(self, pixel, time_step):
        rospy.init_node('run_robot')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.path = pixel
        self.time_step = time_step
        #self.converting_factor = 0.0002645833
        self.converting_factor = 0.05
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.linear_vector = Vector3(x=0.0,y=0.0,z=0.0)
        self.angular_vector = Vector3(x=0.0,y=0.0,z=0.0)
        self.old_direction = (1,0)
        self.commands = []
        self.rate = rospy.Rate(5)
        self.commands = []

    def linear_approximation(self, num):
        """
        :param num: number of pixels skipped
        :return: linear approximation of the path with vectors
        """
        vectors_heading = {}
        for i in range(int(len(self.path) / num) - 1):
            x_start = self.path[num * i][0]
            y_start = self.path[num * i][1]
            x_end = self.path[num * (i + 1)][0]
            y_end = self.path[num * (i + 1)][1]
            heading = (x_end - x_start, y_end - y_start)
            vectors_heading[i] = heading
        return vectors_heading

    def get_distance_direction(self, num):
        """
        :param num: number of pixels skipped
        :return: return distance and direction of each vector command
        """
        headings = self.linear_approximation(num)
        distances = {}
        directions = {}
        for i in range(len(headings)):
            x = headings[i][0]
            y = headings[i][1]

            # get distance and direction
            distance = math.sqrt(x ** 2 + y ** 2)
            distances[i] = distance
            direction = (1 / distance * x, 1 / distance * y)
            directions[i] = direction
        #print('dist, dir:', distances, directions)
        return distances, directions

    def get_velocity_commands(self, num):
        """
        :param num: number of pixels skipped
        :return: return a list of commands with angular and linear velocity
        """
        distances, directions = self.get_distance_direction(num)
        commands = []
        for i in range(len(distances)):
            # find angle to turn
            self.new_direction = directions[i]
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
            #print('theta',theta)

            # calculate linear velocity
            self.linear_velocity = distances[i] / self.time_step * self.converting_factor

            # calculate angular velocity
            self.angular_velocity = theta / self.time_step

            #print('lin velocity', self.linear_velocity)
            #print('ang velocity', self.angular_velocity)

            self.commands = [self.linear_velocity, self.angular_velocity]
            #print('commands:', commands)
            '''
            for i in range(len(self.commands)):
                print('commands list: ', self.commands)
                self.lin_vec = Vector3(self.linear_velocity,0,0)
                self.ang_vec = Vector3(0,0,self.angular_velocity)
                self.pub.publish(Twist(linear=self.linear_velocity, angular=self.angular_velocity))
            '''

    def run_robot(self):
        # Add to vectors
        for i in range(len(self.commands)):
            #print('comms list:', self.commands[i])
            self.linear_vector = Vector3(self.linear_velocity,0,0)
            self.angular_vector = Vector3(0,0,self.angular_velocity)

            print('lin vector',self.linear_vector)
            print('ang vector', self.angular_vector)

            # Publish
            self.pub.publish(Twist(linear=self.linear_vector, angular=self.angular_vector))
            
            # add to commands
            self.commands.append(self.angular_velocity)
            self.commands.append(self.linear_velocity)

            #print('lin_vel:',self.linear_velocity)
            #print('ang_vel:',self.angular_velocity)

            # update old direction
            self.old_direction = self.new_direction
            self.rate.sleep()

            #return commands
            #print('commands:', self.commands)
   
if __name__ == '__main__':
    while not rospy.is_shutdown():
        nav = astar()
        coordinates = nav.actualAStar((585,190),(665,225),"../maps/ac109_goodmap.png") #Coords for round trashcan to corner of box
        robot = navstar(coordinates, 1)
        robot.get_velocity_commands(10)
        robot.run_robot()
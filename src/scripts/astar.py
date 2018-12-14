#!/usr/bin/env python
"""
A script for running A* and outputting a safe path given a set of coordinates
"""

from __future__ import print_function
import rospy
from std_msgs.msg import String
from PIL import Image, ImageFont, ImageDraw
import ast
import textwrap
import numpy as np
import time
import pdb

class astar():
    def __init__(self):
        rospy.init_node('astar')
        self.sub = rospy.Subscriber('/pixels_string', String, self.pixel_callback)
        self.rate = rospy.Rate(2)
        
        # Hardcode some initial values
        self.init_x_coord = 660
        self.init_y_coord = 600
        self.dest_x_coord = 680
        self.dest_y_coord = 495

        # Set initial map size
        self.x_size = 0
        self.y_size = 0
    
    def pixel_callback(self, msg):
        """
        Inputs: Coordinate data from ROS topic for starting and ending nodes
        Outputs: initial x and y coordinates
        """
        eval = ast.literal_eval(msg.data)
        #print('evaluated msg:', eval)

        self.init_x_coord = int(eval[0][0])
        self.init_y_coord = int(eval[0][1])

        self.dest_x_coord = int(eval[1][0])
        self.dest_y_coord = int(eval[1][1])

        #print(self.init_x_coord, self.init_y_coord)
        #print(self.dest_x_coord, self.dest_y_coord)

    def run_astar(self,init_pixel,dest_pixel,map):
        """
        Inputs: starting pixel, finishing pixel, map location
        Outputs: Draws a line of the path the robot needs to take to get to the defined goal
        """
        robot_dims = 3 # Set the radius of the robot in pixels
        map = Image.open(map) # create a map image
        
        # Get dimensions of the map
        self.x_size = map.size[0]
        #print('x_size:', self.x_size)
        self.y_size = map.size[1]
        #print('y_size:', self.y_size)

        # Offset y pixels to account for different origins
        offset_init_y_pixel = (self.x_size - init_pixel[1])
        init_pixel = (init_pixel[0], offset_init_y_pixel)
        print('Initial Node:', init_pixel)
        offset_dest_y_pixel = (self.x_size - dest_pixel[1])
        dest_pixel = (dest_pixel[0], offset_dest_y_pixel)
        print('Destination Node:', dest_pixel)

        #Set initial A* variable values (https://en.wikipedia.org/wiki/A*_search_algorithm)
        G = {} # cost of path from start node to n(next node on path)
        H = {} # heuristic = cost of cheapest path from n to goal node
        F = {} # Optimization such that f(n) = g(n) + h(n)

        obstacles = {} # initialize dict for obstacles

        # Params for filling obstacle dict
        for i in range(self.x_size):
            for j in range(self.y_size):
                if(map.getpixel((i,j))[1] < 200): #TODO: 200 currently hardcoded as pixel value, should change
                    obstacles[(i,j)] = True
        G[init_pixel] = 0
        old_coord = init_pixel # initializes the starting coordinate

        # Check for initial and destination pixels in obstacles dict
        if (init_pixel in obstacles):
            raise ValueError('Once a landlubber, always a landlubber')
        if(dest_pixel in obstacles):
            raise ValueError('You beached the ship')
        
        uncleared_nodes = {} # create list of nodes to check
        been_handled = {init_pixel:True} # create list of checked nodes (including init_pixel)
        destFlag = False # Flag for whether destination node has been reached

        # Run these jewels
        while(not destFlag):
            adjacent_coords = [(old_coord[0]+1,old_coord[1]),(old_coord[0]-1,old_coord[1]),(old_coord[0],old_coord[1]+1),(old_coord[0],old_coord[1]-1)] # define the adjacent_coords to check
            for i in adjacent_coords: # Check all adjacent nodes
                if (not i in been_handled): # If you haven't checked the node yet
                    if(i[0] > 0 and i[0] < self.x_size and i[1] > 0 and i[1] < self.y_size): # making sure the node is on the
                        available_node = self.avoid_obstacles(i,robot_dims,obstacles)
                        if(available_node): # Change cost if pixel is allowed
                            G[i] = G[old_coord] + 1 # Yeah that's right, it got more expensive now
                            H[i] = abs(dest_pixel[1] - i[1]) + abs(dest_pixel[0] - i[0]) # h = euclidean distance between current node and dest node
                            F[i] = H[i] + G[i] # Run that good good f(n) function
                            uncleared_nodes[i] = F[i] # Add f value to open list (to minimize f value)
                        else:
                            been_handled[i] = True # Add pixel to already checked list if it's not available
            if(dest_pixel in uncleared_nodes): # oh hey look at that, we're here
                destFlag = True # better tell someone we made it
            min_val = min(uncleared_nodes.itervalues()) # Get minimum F value
            goodKeys = [k for k, v in uncleared_nodes.iteritems() if v == min_val]
            HDict = {}
            for i in goodKeys:
                HDict[i] = H[i]
            old_coord = min(HDict, key=HDict.get) #  Get minimum H value (should be closer to destination node)
            uncleared_nodes.pop(old_coord) # pop that old coord right outta here
            been_handled[old_coord] = True # aka add it to the already checked list
        coord_list = self.plot_path(init_pixel,dest_pixel,G,map) # now plot a path
        #print('init_pixel, dest_pixel:', init_pixel, dest_pixel)
        #print('coord list:',coord_list)
        return coord_list

    def plot_path(self,init_pixel,dest_pixel,G,map):
        """
        Input: The initial pixel location, destination pixel location, G, and the map
        Output: A processed version of the map with a path plotted and a list of coordinates needed to travel
        """
        solvestar = map # create a solution map to draw path on
        #Initialize variables
        old_pixel = dest_pixel
        coord_list = [dest_pixel]
        coords = [dest_pixel]
        while(init_pixel != old_pixel): # while we haven't gotten to the init_pixel yet
            coords = [(old_pixel[0]+1,old_pixel[1]),(old_pixel[0]-1,old_pixel[1]),(old_pixel[0],old_pixel[1]+1),(old_pixel[0],old_pixel[1]-1)] # checks the 4 surounding pixels
            G_values = [10000,10000,10000,10000] # initializes their values to 0
            for i in range(len(coords)): # Iterate through our coordinates
                try: # In case we go out of bounds (it happens)
                    G_values[i] = G[coords[i]]
                except:
                    G_values[i] = 10000
            next_coord = coords[G_values.index(min(G_values))] # Add coordinates to check next
            coord_list = [next_coord] + coord_list
            solvestar.putpixel(next_coord,(0,255,255)) # Color our new coord
            old_pixel = next_coord # Set the next_coord to our old_pixel to loop through process
        solvestar.save("../maps/processed/solvestar_goodmap.png") # TODO: ideally this wouldnt be hardcoded but it's late and I'm tired
        #print('coord list:', coord_list)
        return coord_list

    def avoid_obstacles(self,i,robot_dims,obstacles):
        """
        Input: coordinate to check(i), dimensions of robot, coordinates of obstacles
        Output: A boolean of whether the robot can actually reach the pixel or not.
        """
        for a in range(robot_dims): # run through the edges of the square around the robot
            b = 0 # top edge of robot
            #Return false if pixel on edge is in obstacle dict
            if (i[0]+robot_dims/2-a,i[1]+robot_dims/2-b) in obstacles:
                return False
            # Repeat this process for bottom edge of robot too
            b= robot_dims
            if(i[0]+robot_dims/2-a,i[1]+robot_dims/2-b) in obstacles:
                return False
            # Now do the same thing for the right and left edges of the robot
        for b in range(robot_dims):
            a = 0
            if(i[0]+robot_dims/2-a,i[1]+robot_dims/2-b) in obstacles:
                return False
            a = robot_dims
            if(i[0]+robot_dims/2-a,i[1]+robot_dims/2-b) in obstacles:
                return False
        return True # If we're good, return True
    
    # Run astar
    def main(self):
        while 1:
            if self.init_x_coord and self.init_y_coord != None:
                break
            break
        start_time = time.time()
        r = rospy.Rate(2)
        #print(self.init_x_coord, self.init_y_coord)
        coords = self.run_astar((self.init_x_coord, self.init_y_coord), (self.dest_x_coord, self.dest_y_coord), "../maps/finalnightslam_bestmap.png") # TODO: also hardcoded, not great
        run_time = (time.time()-start_time)
        r.sleep()
        print("A* Runtime: %s seconds" % run_time)

if __name__ == '__main__':
    nav = astar()
    nav.main()


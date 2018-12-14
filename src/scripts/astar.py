#!/usr/bin/env python

from __future__ import print_function
from PIL import Image, ImageFont, ImageDraw
import textwrap
import numpy as np
import time
import pdb

class astar():
    def run_astar(self,init_pixel,dest_pixel,map):
        """
        Inputs: starting pixel, finishing pixel, map location
        Outputs: Draws a line of the path the robot needs to take to get to the defined goal
        """
        robot_dims = 2 # determines the radius of the robot in pixels
        map = Image.open(map) # gets the map image
        
        # Get dimensions of the map
        x_size = map.size[0]
        #print('x_size:', x_size)
        y_size = map.size[1]
        #print('y_size:', y_size)

        #Set initial A* variable values
        Gs = {} # uses a dictionary to speed up the process
        Hs = {}
        Fs = {}

        obstacles = {} # initialize dict for obstacles found on map

        # Params for filling obstacle dict
        for i in range(x_size):
            for j in range(y_size):
                if(map.getpixel((i,j))[1] < 200):
                    obstacles[(i,j)] = True
        Gs[init_pixel] = 0
        old_coord = init_pixel # initializes the starting coordinate
        #print('start coord:', old_coord)
        #print('obstacles:', obstacles)

        # Check threshold is correct for obstacle detection
        if (init_pixel in obstacles):
            raise ValueError('initial pixel is an obstacle')
        if(dest_pixel in obstacles):
            raise ValueError('Ending pixel is an obstacle')
        openList = {}
        closedList = {init_pixel:True}
        goalReached = False # a flag to check whether the goal has been reached yet.

        # Run these jewels
        while(not goalReached):
            adjacentCoords = [(old_coord[0]+1,old_coord[1]),(old_coord[0]-1,old_coord[1]),(old_coord[0],old_coord[1]+1),(old_coord[0],old_coord[1]-1)] # define the adjacentCoords to check
            for i in adjacentCoords:
                if(not i in closedList):
                    if(i[0] > 0 and i[0] < x_size and i[1] > 0 and i[1] < y_size): # making sure that the bounds aren't out of range.
                        available = self.self_check(i,robot_dims,obstacles)
                        if(available): # if the pixel is allowed, change it's cost
                            Gs[i] = Gs[old_coord] + 1 # changes the cost
                            Hs[i] = abs(dest_pixel[1] - i[1]) + abs(dest_pixel[0] - i[0]) # change the h value to be the euclidean distance between the point and the finish
                            Fs[i] = Hs[i] + Gs[i] # the F cost as the sum of the past two
                            openList[i] = Fs[i] # the open list contains the F value to make it easier to pick the lowest f value
                        else:
                            closedList[i] = True # If the pixel is not allowed, it is added to the closed list to prevent further checking.
            if(dest_pixel in openList):
                goalReached = True
            min_val = min(openList.itervalues()) # gets the lowest F value in the dictionary of open pixels
            goodKeys = [k for k, v in openList.iteritems() if v == min_val] # gets the pixel(s) with the lowest F value
            HDict = {}
            for i in goodKeys:
                HDict[i] = Hs[i]
            old_coord = min(HDict, key=HDict.get) # from here, gets the pixel with the lowest H value (most likely to be close to the goal)
            openList.pop(old_coord)
            closedList[old_coord] = True
        ListOfCoordinates = self.plot_path(init_pixel,dest_pixel,Gs,map)
        #print('coord list:',ListOfCoordinates)
        return ListOfCoordinates

    def plot_path(self,init_pixel,dest_pixel,Gs,map):
        solution = map
        oldPixel = dest_pixel
        ListOfCoordinates = [dest_pixel]
        coords = [dest_pixel]
        while(init_pixel != oldPixel): # while we haven't gotten to the init_pixel yet
            coords = [(oldPixel[0]+1,oldPixel[1]),(oldPixel[0]-1,oldPixel[1]),(oldPixel[0],oldPixel[1]+1),(oldPixel[0],oldPixel[1]-1)] # checks the 4 surounding pixels
            Gvalues = [10000,10000,10000,10000] # initializes their values to 0
            for i in range(len(coords)): # puts their values in
                try: # has a tendency to go out of bounds (it can't get to pixel)
                    Gvalues[i] = Gs[coords[i]]
                except:
                    Gvalues[i] = 10000
            coordToGoTo = coords[Gvalues.index(min(Gvalues))] # adds the coordinate that we want to go to next.
            ListOfCoordinates = [coordToGoTo] + ListOfCoordinates
            solution.putpixel(coordToGoTo,(0,255,0)) # makes that coordinate red
            oldPixel = coordToGoTo # does it again
        solution.save("../maps/processed/solution_goodmap.png") # TODO: ideally this wouldnt be hardcoded
        #print('coord list:', ListOfCoordinates)
        return ListOfCoordinates

    def self_check(self,i,robot_dims,obstacles):
        """
        Checks the edges of the robot to determine if the pixel in question is a pixel that is allowed to be travelled
        input: i, the coordinate in question, robot_dims, the size of the robot in pixels, obstacles, the coordinates of all the obstacles
        output: boolean, whether the pixel is available or not.
        """
        for j in range(robot_dims): # run through the edges of the square around the robot
            k = 0 # top edge
            if (i[0]+robot_dims/2-j,i[1]+robot_dims/2-k) in obstacles: # if a pixel on the edge is in the obstacle dictionary
                return False # return False as the pixel is no longer allowed
            k= robot_dims # repeat for bottom edge
            if(i[0]+robot_dims/2-j,i[1]+robot_dims/2-k) in obstacles:
                return False
        for k in range(robot_dims): # doing the same as above, but for the right and left edges
            j = 0
            if(i[0]+robot_dims/2-j,i[1]+robot_dims/2-k) in obstacles:
                return False
            j = robot_dims
            if(i[0]+robot_dims/2-j,i[1]+robot_dims/2-k) in obstacles:
                return False
        return True

if __name__ == '__main__':
    start_time = time.time()
    nav = astar()
    #init_x_coord = int(raw_input('init x coord: '))
    #init_y_coord = int(raw_input('init y coord: '))
    #dest_x_coord = int(raw_input('dest x coord: '))
    #dest_y_coord = int(raw_input('dest y coord: '))
    #coordinates = nav.run_astar((init_x_coord, init_y_coord), (dest_x_coord, dest_y_coord), "../maps/ac109_goodmap.png")
    #coordinates = nav.run_astar((580,325),(585,190),"../maps/ac109_goodmap.png") #Coords for corner of map to round trashcan
    coordinates = nav.run_astar((585,190),(665,225),"../maps/ac109_goodmap.png") #Coords for round trashcan to corner of box
    run_time = (time.time()-start_time)
    print("--- %s seconds ---" % run_time)
